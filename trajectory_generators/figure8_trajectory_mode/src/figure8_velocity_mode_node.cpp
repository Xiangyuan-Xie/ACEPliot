#include <algorithm>
#include <memory>
#include <string>

#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <rclcpp/rclcpp.hpp>

#include <figure8_trajectory_mode/figure8_generator.hpp>
#include <trajectory_generator_utils/offboard_conversion.hpp>
#include <trajectory_generator_utils/state_provider.hpp>

#include "figure8_mode_utils.hpp"

namespace
{
class Figure8VelocityModeNode : public rclcpp::Node
{
public:
  Figure8VelocityModeNode()
  : rclcpp::Node("figure8_velocity_mode"),
    px4_context_(*this, this->declare_parameter<std::string>("topic_namespace_prefix", ""))
  {
    publish_rate_hz_ = std::max(1.0, this->declare_parameter<double>("publish_rate_hz", 50.0));
    publish_path_ = this->declare_parameter<bool>("publish_path", true);

    const StateProviderConfig state_config{
      this->declare_parameter<bool>("use_ros2_odom", false),
      this->declare_parameter<std::string>("odom_topic", "/odom")
    };
    state_provider_ = std::make_unique<TrajectoryStateProvider>(*this, px4_context_, state_config);

    const std::string offboard_control_mode_topic = this->declare_parameter<std::string>(
      "offboard_control_mode_topic",
      "/fmu/in/offboard_control_mode" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::OffboardControlMode>());
    const std::string trajectory_setpoint_topic = this->declare_parameter<std::string>(
      "trajectory_setpoint_topic",
      "/fmu/in/trajectory_setpoint" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::TrajectorySetpoint>());
    path_topic_ = this->declare_parameter<std::string>(
      "path_topic", "/trajectory_generators/path");

    offboard_control_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_control_mode_topic, 10);
    trajectory_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_setpoint_topic, 10);
    if (publish_path_) {
      path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_, 1);
    }

    Figure8Generator::Figure8Params params;
    params.period_s = static_cast<float>(
      this->declare_parameter<double>("figure8.period_s", params.period_s));
    params.A = static_cast<float>(this->declare_parameter<double>("figure8.A", params.A));
    params.B = static_cast<float>(this->declare_parameter<double>("figure8.B", params.B));
    params.yaw_rate = static_cast<float>(
      this->declare_parameter<double>("figure8.yaw_rate", params.yaw_rate));
    params.target_height = static_cast<float>(
      this->declare_parameter<double>("figure8.target_height", params.target_height));
    params.max_speed = static_cast<float>(
      this->declare_parameter<double>("figure8.max_speed", params.max_speed));
    params.max_acceleration = static_cast<float>(
      this->declare_parameter<double>("figure8.max_acceleration", params.max_acceleration));
    params.transition_time = static_cast<float>(
      this->declare_parameter<double>("figure8.transition_time", params.transition_time));
    params.decel_time = static_cast<float>(
      this->declare_parameter<double>("figure8.decel_time", params.decel_time));
    params.loops_to_run = this->declare_parameter<int>("figure8.loops_to_run", params.loops_to_run);
    params.max_phase_accel = static_cast<float>(
      this->declare_parameter<double>("figure8.max_phase_accel", params.max_phase_accel));
    params.climb_speed = static_cast<float>(
      this->declare_parameter<double>("figure8.climb_speed", params.climb_speed));
    params.climb_accel = static_cast<float>(
      this->declare_parameter<double>("figure8.climb_accel", params.climb_accel));
    params.land_speed = static_cast<float>(
      this->declare_parameter<double>("figure8.land_speed", params.land_speed));
    params.land_accel = static_cast<float>(
      this->declare_parameter<double>("figure8.land_accel", params.land_accel));
    generator_.setParams(params);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Figure8VelocityModeNode::onTimer, this));
  }

private:
  void onTimer()
  {
    TrajectoryGeneratorState state;
    if (!state_provider_->getState(state)) {
      return;
    }

    if (!initialized_) {
      generator_.reset(state);
      initialized_ = true;
    }

    const float dt_s = static_cast<float>(1.0 / publish_rate_hz_);
    const TrajectorySample sample = makeVelocityModeSample(generator_.step(dt_s, state));

    offboard_control_mode_pub_->publish(makeOffboardControlMode(sample));
    trajectory_setpoint_pub_->publish(makeTrajectorySetpoint(sample));

    if (publish_path_ && path_pub_) {
      path_pub_->publish(generator_.getFullPathForVisualization(this->now()));
    }
  }

  px4_ros2::Context px4_context_;
  double publish_rate_hz_{50.0};
  bool publish_path_{true};
  bool initialized_{false};
  std::string path_topic_;
  Figure8Generator generator_;
  std::unique_ptr<TrajectoryStateProvider> state_provider_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Figure8VelocityModeNode>());
  rclcpp::shutdown();
  return 0;
}
