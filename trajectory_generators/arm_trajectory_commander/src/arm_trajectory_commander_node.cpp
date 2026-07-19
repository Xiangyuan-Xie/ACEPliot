#include <arm_trajectory_commander/arm_trajectory.hpp>
#include <arm_trajectory_commander/clock_fallback.hpp>
#include <arm_trajectory_commander/vehicle_position_metrics.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class ArmTrajectoryCommanderNode : public rclcpp::Node
{
public:
  ArmTrajectoryCommanderNode()
  : rclcpp::Node("arm_trajectory_commander_node"),
    use_sim_time_(readUseSimTime()),
    allow_wall_time_without_clock_(readAllowWallTimeWithoutClock()),
    profile_(readArmConfig()),
    sync_handshake_(readSyncConfig()),
    playback_(profile_),
    clock_fallback_(use_sim_time_, allow_wall_time_without_clock_),
    steady_timer_clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME))
  {
    publish_rate_hz_ = std::max(1.0, this->declare_parameter<double>("publish_rate_hz", 100.0));

    const std::string arm_command_topic = this->declare_parameter<std::string>(
      "arm_command_topic", "/ace_leader/arm/command");
    const std::string sync_mode_topic = this->declare_parameter<std::string>(
      "sync_mode_topic", "/ace_leader/arm/sync_mode");
    const std::string gripper_command_topic = this->declare_parameter<std::string>(
      "gripper_command_topic", "/ace_leader/gripper/command");
    const std::string follower_arm_state_topic = this->declare_parameter<std::string>(
      "follower_arm_state_topic", "/ace_follower/arm/state");
    const std::string follower_sync_status_topic = this->declare_parameter<std::string>(
      "follower_sync_status_topic", "/ace_follower/arm/sync_status");
    const std::string follower_gripper_state_topic = this->declare_parameter<std::string>(
      "follower_gripper_state_topic", "/ace_follower/gripper/state");
    const VehiclePositionSource vehicle_position_source = parseVehiclePositionSource(
      this->declare_parameter<std::string>(
        "vehicle_position_source", use_sim_time_ ? "odometry_pose" : "pose_stamped"));
    const std::string vehicle_pose_topic = this->declare_parameter<std::string>(
      "vehicle_pose_topic", "/xxy/pose");
    const std::string vehicle_odometry_topic = this->declare_parameter<std::string>(
      "vehicle_odometry_topic", "/acesim/vehicle/odometry");

    arm_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(arm_command_topic, 10);
    sync_mode_pub_ = this->create_publisher<std_msgs::msg::String>(sync_mode_topic, 10);
    gripper_command_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>(gripper_command_topic, 10);
    follower_arm_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      follower_arm_state_topic, 10,
      std::bind(&ArmTrajectoryCommanderNode::onFollowerArmState, this, std::placeholders::_1));
    follower_sync_status_sub_ = this->create_subscription<std_msgs::msg::String>(
      follower_sync_status_topic, 10,
      std::bind(&ArmTrajectoryCommanderNode::onFollowerSyncStatus, this, std::placeholders::_1));
    follower_gripper_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      follower_gripper_state_topic, 10,
      std::bind(&ArmTrajectoryCommanderNode::onFollowerGripperState, this, std::placeholders::_1));
    const auto vehicle_sensor_qos = rclcpp::SensorDataQoS();
    if (vehicle_position_source == VehiclePositionSource::PoseStamped) {
      vehicle_position_topic_ = vehicle_pose_topic;
      vehicle_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        vehicle_pose_topic, vehicle_sensor_qos,
        std::bind(&ArmTrajectoryCommanderNode::onVehiclePose, this, std::placeholders::_1));
    } else {
      vehicle_position_topic_ = vehicle_odometry_topic;
      vehicle_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        vehicle_odometry_topic, vehicle_sensor_qos,
        std::bind(&ArmTrajectoryCommanderNode::onVehicleOdometry, this, std::placeholders::_1));
    }
    if (use_sim_time_) {
      sim_clock_topic_ =
        this->declare_parameter<std::string>("sim_clock_topic", "/acesim/clock");
      sim_clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        sim_clock_topic_, rclcpp::ClockQoS(),
        std::bind(&ArmTrajectoryCommanderNode::onSimClock, this, std::placeholders::_1));
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = rclcpp::create_timer(
      this->get_node_base_interface(),
      this->get_node_timers_interface(),
      steady_timer_clock_,
      rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(period)),
      std::bind(&ArmTrajectoryCommanderNode::onTimer, this));
  }

private:
  ArmTrajectoryConfig readArmConfig()
  {
    ArmTrajectoryConfig config;
    config.joint_names = this->declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4"});
    config.segment_durations_s = this->declare_parameter<std::vector<double>>(
      "segment_durations_s", std::vector<double>{2.0, 2.0, 2.0});
    config.positions = this->declare_parameter<std::vector<double>>(
      "positions",
      std::vector<double>{
      0.0, 0.0, 0.0, 0.0,
      0.2, -0.2, 0.15, -0.15,
      0.0, 0.0, 0.0, 0.0,
    });
    config.max_joint_velocity_rad_s =
      this->declare_parameter<double>("max_joint_velocity_rad_s", 1.5);
    config.loop_count = this->declare_parameter<int>("loop_count", 1);
    config.publish_gripper = this->declare_parameter<bool>("publish_gripper", true);
    config.gripper_joint_name =
      this->declare_parameter<std::string>("gripper_joint_name", "joint_5");
    config.gripper_positions = this->declare_parameter<std::vector<double>>(
      "gripper_positions", std::vector<double>{0.0, 0.5, 0.0});
    config.max_gripper_velocity_rad_s =
      this->declare_parameter<double>("max_gripper_velocity_rad_s", 1.0);
    return config;
  }

  ArmSyncConfig readSyncConfig()
  {
    ArmSyncConfig config;
    config.enable_sync_handshake =
      this->declare_parameter<bool>("enable_sync_handshake", true);
    config.follower_state_timeout_s =
      this->declare_parameter<double>("follower_state_timeout_s", 0.5);
    config.sync_status_timeout_s =
      this->declare_parameter<double>("sync_status_timeout_s", 0.5);
    config.ready_dwell_s = this->declare_parameter<double>("ready_dwell_s", 0.2);
    config.auto_start_tracking =
      this->declare_parameter<bool>("auto_start_tracking", true);
    config.require_follower_state_before_tracking =
      this->declare_parameter<bool>("require_follower_state_before_tracking", !use_sim_time_);
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
        "Sim clock topic '%s' is active; arm trajectory uses sim time.",
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

  void onSimClock(const rosgraph_msgs::msg::Clock::SharedPtr msg)
  {
    latest_sim_time_ = rclcpp::Time(msg->clock, RCL_ROS_TIME);
  }

  void onFollowerArmState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.empty()) {
      return;
    }
    latest_follower_arm_position_ = msg->position;
    sync_handshake_.notifyFollowerState(effectiveNowS());
  }

  void onFollowerSyncStatus(const std_msgs::msg::String::SharedPtr msg)
  {
    sync_handshake_.notifyFollowerSyncStatus(msg->data, effectiveNowS());
  }

  void onFollowerGripperState(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.empty()) {
      return;
    }
    latest_follower_gripper_position_ = msg->position.front();
  }

  void onVehiclePose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
      stamp = effectiveStamp(effectiveNowS());
    }

    recordVehicleMetrics(
      vehicle_position_metrics_.update(
        VehiclePositionSample{
      std::array<double, 3>{
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z,
      },
      stamp.seconds()}));
  }

  void onVehicleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
      stamp = effectiveStamp(effectiveNowS());
    }

    recordVehicleMetrics(
      vehicle_position_metrics_.update(
        VehiclePositionSample{
      acesimNwuPositionToEnu(
        std::array<double, 3>{
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z,
      }),
      stamp.seconds()}));
  }

  void publishSyncMode(const std::string & mode)
  {
    std_msgs::msg::String sync_msg;
    sync_msg.data = mode;
    sync_mode_pub_->publish(sync_msg);
  }

  void recordVehicleMetrics(const HoverPositionMetrics & metrics)
  {
    latest_vehicle_metrics_ = metrics;
    if (collect_vehicle_summary_) {
      vehicle_position_summary_.update(metrics);
    }
  }

  void reportVehiclePositionMetrics(double now_s)
  {
    if (now_s < next_vehicle_metrics_report_s_) {
      return;
    }
    next_vehicle_metrics_report_s_ = now_s + 1.0;

    if (!latest_vehicle_metrics_.has_value()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting for vehicle hover position metrics on '%s'.",
        vehicle_position_topic_.c_str());
      return;
    }

    const HoverPositionMetrics & metrics = latest_vehicle_metrics_.value();
    const double sample_age_s = std::max(0.0, now_s - metrics.sample_time_s);
    RCLCPP_INFO(
      this->get_logger(),
      "Vehicle hover metrics [%s]: position_enu_m=(%.3f, %.3f, %.3f), "
      "drift_enu_m=(%.3f, %.3f, %.3f), drift_xy_m=%.3f, drift_xyz_m=%.3f, "
      "sample_age_s=%.3f",
      vehicle_position_topic_.c_str(),
      metrics.position_enu_m[0],
      metrics.position_enu_m[1],
      metrics.position_enu_m[2],
      metrics.drift_enu_m[0],
      metrics.drift_enu_m[1],
      metrics.drift_enu_m[2],
      metrics.drift_xy_m,
      metrics.drift_xyz_m,
      sample_age_s);
  }

  void reportVehiclePositionSummary()
  {
    if (reported_vehicle_summary_) {
      return;
    }
    reported_vehicle_summary_ = true;
    vehicle_position_summary_.endSegment();

    const std::optional<HoverPositionSummary> summary = vehicle_position_summary_.summary();
    if (!summary.has_value()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Vehicle hover summary unavailable [%s]: no vehicle position samples were collected "
        "during tracking.",
        vehicle_position_topic_.c_str());
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Vehicle hover summary [%s]: duration_s=%.3f, samples=%zu, "
      "mean_position_enu_m=(%.3f, %.3f, %.3f), "
      "mean_drift_enu_m=(%.3f, %.3f, %.3f), mean_drift_xy_m=%.3f, "
      "mean_drift_xyz_m=%.3f",
      vehicle_position_topic_.c_str(),
      summary->duration_s,
      summary->sample_count,
      summary->mean_position_enu_m[0],
      summary->mean_position_enu_m[1],
      summary->mean_position_enu_m[2],
      summary->mean_drift_enu_m[0],
      summary->mean_drift_enu_m[1],
      summary->mean_drift_enu_m[2],
      summary->mean_drift_xy_m,
      summary->mean_drift_xyz_m);
  }

  void onTimer()
  {
    const bool had_ros_clock = clock_fallback_.hasRosClock();
    const double now_s = effectiveNowS();
    const bool switched_to_ros_clock = !had_ros_clock && clock_fallback_.hasRosClock();
    reportVehiclePositionMetrics(now_s);

    if (trajectory_finished_) {
      publishSyncMode("stop");
      reportVehiclePositionSummary();
      RCLCPP_INFO(
        this->get_logger(),
        "Arm trajectory stopped; shutting down arm trajectory commander node.");
      rclcpp::shutdown();
      return;
    }

    const rclcpp::Time stamp = effectiveStamp(now_s);
    const ArmSyncUpdate sync_update = sync_handshake_.update(now_s);

    publishSyncMode(sync_update.leader_mode);

    if (!sync_update.commands_allowed) {
      if (collect_vehicle_summary_) {
        collect_vehicle_summary_ = false;
        vehicle_position_summary_.endSegment();
      }
      return;
    }
    if (!collect_vehicle_summary_) {
      collect_vehicle_summary_ = true;
      vehicle_position_summary_.endSegment();
    }

    if (sync_update.tracking_started || !playback_.active() || switched_to_ros_clock) {
      if (latest_follower_arm_position_.has_value()) {
        playback_.startFromFollowerState(
          now_s, *latest_follower_arm_position_, latest_follower_gripper_position_);
        if (profile_.firstGripperPosition().has_value() && !playback_.usedFollowerGripperState()) {
          RCLCPP_WARN_ONCE(
            this->get_logger(),
            "Follower gripper state unavailable; starting gripper at first trajectory waypoint.");
        }
      } else {
        RCLCPP_WARN_ONCE(
          this->get_logger(),
          "Follower arm state unavailable; starting arm trajectory at first YAML waypoint.");
        playback_.startAt(now_s);
      }
    }

    const ArmTrajectorySample sample = playback_.sample(now_s);

    sensor_msgs::msg::JointState arm_msg;
    arm_msg.header.stamp = stamp;
    arm_msg.name = sample.joint_names;
    arm_msg.position = sample.positions;
    arm_msg.velocity = sample.velocities;
    arm_msg.effort = sample.efforts;
    arm_command_pub_->publish(arm_msg);

    if (sample.gripper.has_value()) {
      sensor_msgs::msg::JointState gripper_msg;
      gripper_msg.header.stamp = stamp;
      gripper_msg.name = {sample.gripper->name};
      gripper_msg.position = {sample.gripper->position};
      gripper_msg.velocity = {sample.gripper->velocity};
      gripper_msg.effort = {sample.gripper->effort};
      gripper_command_pub_->publish(gripper_msg);
    }

    if (sample.finished) {
      collect_vehicle_summary_ = false;
      vehicle_position_summary_.endSegment();
      reportVehiclePositionSummary();
      trajectory_finished_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Arm trajectory finished; publishing stop sync mode before shutdown.");
    }
  }

  double publish_rate_hz_{100.0};
  bool use_sim_time_{false};
  bool allow_wall_time_without_clock_{true};
  bool trajectory_finished_{false};
  bool collect_vehicle_summary_{false};
  bool reported_vehicle_summary_{false};
  ArmTrajectoryProfile profile_;
  ArmSyncHandshake sync_handshake_;
  ArmTrajectoryPlayback playback_;
  ClockFallback clock_fallback_;
  rclcpp::Clock::SharedPtr steady_timer_clock_;
  std::string sim_clock_topic_{"/acesim/clock"};
  std::string vehicle_position_topic_{"/xxy/pose"};
  double next_vehicle_metrics_report_s_{0.0};
  bool warned_clock_fallback_{false};
  bool reported_ros_clock_{false};
  std::optional<rclcpp::Time> latest_sim_time_;
  std::optional<HoverPositionMetrics> latest_vehicle_metrics_;
  VehiclePositionMetricsTracker vehicle_position_metrics_;
  VehiclePositionSummaryAccumulator vehicle_position_summary_;
  std::optional<std::vector<double>> latest_follower_arm_position_;
  std::optional<double> latest_follower_gripper_position_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_mode_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_command_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_arm_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr follower_sync_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_gripper_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sim_clock_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTrajectoryCommanderNode>());
  rclcpp::shutdown();
  return 0;
}
