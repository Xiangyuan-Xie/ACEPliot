#include <px4_velocity_commander/acesim_odometry_measurement.hpp>
#include <px4_velocity_commander/clock_fallback.hpp>
#include <px4_velocity_commander/mocap_velocity_estimator.hpp>
#include <px4_velocity_commander/offboard_conversion.hpp>
#include <px4_velocity_commander/velocity_profile.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace
{
geometry_msgs::msg::TwistStamped makeTwistStamped(
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::array<double, 3> & linear_enu_m_s,
  double yaw_rate_enu_rad_s)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.twist.linear.x = linear_enu_m_s[0];
  msg.twist.linear.y = linear_enu_m_s[1];
  msg.twist.linear.z = linear_enu_m_s[2];
  msg.twist.angular.z = yaw_rate_enu_rad_s;
  return msg;
}

uint64_t stampToMicroseconds(const rclcpp::Time & stamp)
{
  return static_cast<uint64_t>(stamp.nanoseconds() / 1000);
}
}  // namespace

class Px4VelocityCommanderNode : public rclcpp::Node
{
public:
  Px4VelocityCommanderNode()
  : rclcpp::Node("px4_velocity_commander_node"),
    use_sim_time_(readUseSimTime()),
    allow_wall_time_without_clock_(readAllowWallTimeWithoutClock()),
    profile_(readProfileConfig()),
    estimator_(readEstimatorConfig()),
    clock_fallback_(use_sim_time_, allow_wall_time_without_clock_),
    steady_timer_clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
  {
    publish_rate_hz_ = std::max(1.0, this->declare_parameter<double>("publish_rate_hz", 50.0));
    measurement_source_ = parseMeasurementSource(
      this->declare_parameter<std::string>("measurement_source", "pose_stamped"));
    frame_id_ = this->declare_parameter<std::string>("velocity_frame_id", "world");

    const std::string offboard_control_mode_topic = this->declare_parameter<std::string>(
      "offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    const std::string trajectory_setpoint_topic = this->declare_parameter<std::string>(
      "trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint");
    const std::string command_velocity_topic = this->declare_parameter<std::string>(
      "command_velocity_topic", "/px4_velocity_commander/command_velocity");
    const std::string measured_velocity_topic = this->declare_parameter<std::string>(
      "measured_velocity_topic", "/px4_velocity_commander/measured_velocity");
    const std::string velocity_error_topic = this->declare_parameter<std::string>(
      "velocity_error_topic", "/px4_velocity_commander/velocity_error");

    offboard_control_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_control_mode_topic, 10);
    trajectory_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_setpoint_topic, 10);
    command_velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>(command_velocity_topic, 10);
    measured_velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>(measured_velocity_topic, 10);
    velocity_error_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistStamped>(velocity_error_topic, 10);

    if (use_sim_time_) {
      sim_clock_topic_ =
        this->declare_parameter<std::string>("sim_clock_topic", "/acesim/clock");
      sim_clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        sim_clock_topic_, rclcpp::ClockQoS(),
        std::bind(&Px4VelocityCommanderNode::onSimClock, this, std::placeholders::_1));
    }

    if (measurement_source_ == MeasurementSource::PoseStamped) {
      const std::string mocap_pose_topic =
        this->declare_parameter<std::string>("mocap_pose_topic", "xxy/pose");
      mocap_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        mocap_pose_topic, 10,
        std::bind(&Px4VelocityCommanderNode::onMocapPose, this, std::placeholders::_1));
    } else {
      const std::string measured_odometry_topic = this->declare_parameter<std::string>(
        "measured_odometry_topic", "/acesim/vehicle/odometry");
      measured_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        measured_odometry_topic, 10,
        std::bind(&Px4VelocityCommanderNode::onMeasuredOdometry, this, std::placeholders::_1));
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      steady_timer_clock_,
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(period)),
      std::bind(&Px4VelocityCommanderNode::onTimer, this));
  }

private:
  VelocityProfileConfig readProfileConfig()
  {
    VelocityProfileConfig config;
    config.durations_s = this->declare_parameter<std::vector<double>>(
      "profile.durations_s", std::vector<double>{2.0, 2.0, 2.0});
    config.vx_m_s = this->declare_parameter<std::vector<double>>(
      "profile.vx_m_s", std::vector<double>{0.25, 0.0, 0.0});
    config.vy_m_s = this->declare_parameter<std::vector<double>>(
      "profile.vy_m_s", std::vector<double>{0.0, 0.25, 0.0});
    config.vz_m_s = this->declare_parameter<std::vector<double>>(
      "profile.vz_m_s", std::vector<double>{0.0, 0.0, 0.0});
    config.yaw_rate_rad_s = this->declare_parameter<std::vector<double>>(
      "profile.yaw_rate_rad_s", std::vector<double>{0.0, 0.0, 0.0});
    config.loop = this->declare_parameter<bool>("profile.loop", false);
    config.max_linear_speed_m_s =
      this->declare_parameter<double>("profile.max_linear_speed_m_s", 2.0);
    config.max_yaw_rate_rad_s =
      this->declare_parameter<double>("profile.max_yaw_rate_rad_s", 1.0);
    return config;
  }

  bool readUseSimTime()
  {
    if (this->has_parameter("use_sim_time")) {
      return this->get_parameter("use_sim_time").as_bool();
    }
    return this->declare_parameter<bool>("use_sim_time", false);
  }

  bool readAllowWallTimeWithoutClock()
  {
    return this->declare_parameter<bool>("allow_wall_time_without_clock", true);
  }

  double effectiveNowS()
  {
    const rclcpp::Time ros_now = latest_sim_time_.value_or(this->get_clock()->now());
    const rclcpp::Time steady_now = steady_timer_clock_->now();
    const double now_s = clock_fallback_.effectiveNowS(ros_now.seconds(), steady_now.seconds());
    if (clock_fallback_.usingFallback() && !warned_clock_fallback_) {
      RCLCPP_WARN(
        this->get_logger(),
        "use_sim_time is true, but sim clock topic '%s' has not advanced; using steady-time "
        "fallback. Start the configured clock publisher or set allow_wall_time_without_clock:=false "
        "to require sim time.",
        sim_clock_topic_.c_str());
      warned_clock_fallback_ = true;
    }
    if (clock_fallback_.hasRosClock() && warned_clock_fallback_ && !reported_ros_clock_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Sim clock topic '%s' is active; PX4 velocity commander uses sim time.",
        sim_clock_topic_.c_str());
      reported_ros_clock_ = true;
    }
    return now_s;
  }

  rclcpp::Time effectiveStamp(double now_s)
  {
    if (!clock_fallback_.usingFallback()) {
      return latest_sim_time_.value_or(this->get_clock()->now());
    }
    return rclcpp::Time(static_cast<int64_t>(now_s * 1e9), RCL_ROS_TIME);
  }

  MocapVelocityEstimatorConfig readEstimatorConfig()
  {
    MocapVelocityEstimatorConfig config;
    config.min_dt_s = this->declare_parameter<double>("velocity_estimator.min_dt_s", 0.002);
    config.max_dt_s = this->declare_parameter<double>("velocity_estimator.max_dt_s", 0.5);
    config.low_pass_alpha =
      this->declare_parameter<double>("velocity_estimator.low_pass_alpha", 0.5);
    return config;
  }

  void onMocapPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
      stamp = effectiveStamp(effectiveNowS());
    }

    const std::array<double, 3> position_enu_m{
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z,
    };

    const auto velocity = estimator_.update(stamp.seconds(), position_enu_m);
    if (!velocity.has_value()) {
      return;
    }

    last_measured_velocity_ = velocity->linear_enu_m_s;
    last_measured_frame_id_ = msg->header.frame_id.empty() ? frame_id_ : msg->header.frame_id;
    measured_velocity_pub_->publish(
      makeTwistStamped(stamp, last_measured_frame_id_, velocity->linear_enu_m_s, 0.0));
  }

  void onMeasuredOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
      stamp = effectiveStamp(effectiveNowS());
    }

    const std::array<double, 3> position_enu_m = acesimOdometryPositionToEnu(*msg);
    const auto velocity = estimator_.update(stamp.seconds(), position_enu_m);
    if (!velocity.has_value()) {
      return;
    }

    last_measured_velocity_ = velocity->linear_enu_m_s;
    last_measured_frame_id_ = frame_id_;
    measured_velocity_pub_->publish(
      makeTwistStamped(stamp, last_measured_frame_id_, velocity->linear_enu_m_s, 0.0));
  }

  void onSimClock(const rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
    latest_sim_time_ = rclcpp::Time(msg->clock, RCL_ROS_TIME);
  }

  void onTimer()
  {
    const bool had_ros_clock = clock_fallback_.hasRosClock();
    const double now_s = effectiveNowS();
    const bool switched_to_ros_clock = !had_ros_clock && clock_fallback_.hasRosClock();
    const rclcpp::Time now = effectiveStamp(now_s);
    if (!has_start_time_s_ || switched_to_ros_clock) {
      start_time_s_ = now_s;
      has_start_time_s_ = true;
    }
    const double elapsed_s = now_s - start_time_s_;
    const VelocityCommand command = profile_.sample(elapsed_s);
    const uint64_t timestamp_us = stampToMicroseconds(now);

    offboard_control_mode_pub_->publish(makeVelocityOffboardControlMode(timestamp_us));
    trajectory_setpoint_pub_->publish(makeVelocityTrajectorySetpoint(command, timestamp_us));
    command_velocity_pub_->publish(
      makeTwistStamped(now, frame_id_, command.velocity_enu_m_s, command.yaw_rate_enu_rad_s));

    if (last_measured_velocity_.has_value()) {
      std::array<double, 3> error{};
      for (std::size_t i = 0; i < error.size(); ++i) {
        error[i] = command.velocity_enu_m_s[i] - last_measured_velocity_.value()[i];
      }
      velocity_error_pub_->publish(makeTwistStamped(now, last_measured_frame_id_, error, 0.0));
    }
  }

  double publish_rate_hz_{50.0};
  bool use_sim_time_{false};
  bool allow_wall_time_without_clock_{true};
  std::string sim_clock_topic_{"/acesim/clock"};
  MeasurementSource measurement_source_{MeasurementSource::PoseStamped};
  std::string frame_id_{"world"};
  VelocityProfile profile_;
  MocapVelocityEstimator estimator_;
  ClockFallback clock_fallback_;
  rclcpp::Clock::SharedPtr steady_timer_clock_;
  double start_time_s_{0.0};
  bool has_start_time_s_{false};
  bool warned_clock_fallback_{false};
  bool reported_ros_clock_{false};
  std::optional<rclcpp::Time> latest_sim_time_;
  std::optional<std::array<double, 3>> last_measured_velocity_;
  std::string last_measured_frame_id_{"world"};
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr measured_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_error_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr mocap_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr measured_odometry_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sim_clock_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4VelocityCommanderNode>());
  rclcpp::shutdown();
  return 0;
}
