#include <algorithm>
#include <memory>
#include <string>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rc_manual_trajectory_mode/rc_manual_generator.hpp>
#include <trajectory_generator_utils/offboard_conversion.hpp>
#include <trajectory_generator_utils/state_provider.hpp>

namespace
{
class RcManualTrajectoryModeNode : public rclcpp::Node
{
public:
  RcManualTrajectoryModeNode()
  : rclcpp::Node("rc_manual_trajectory_mode"),
    px4_context_(*this, this->declare_parameter<std::string>("topic_namespace_prefix", ""))
  {
    publish_rate_hz_ = std::max(1.0, this->declare_parameter<double>("publish_rate_hz", 50.0));

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

    offboard_control_mode_pub_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(offboard_control_mode_topic, 10);
    trajectory_setpoint_pub_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(trajectory_setpoint_topic, 10);

    RcManualGenerator::RcManualParams params;
    params.v_xy =
      static_cast<float>(this->declare_parameter<double>("rc_manual.v_xy", params.v_xy));
    params.v_up =
      static_cast<float>(this->declare_parameter<double>("rc_manual.v_up", params.v_up));
    params.v_down = static_cast<float>(
      this->declare_parameter<double>("rc_manual.v_down", params.v_down));
    params.acc_xy = static_cast<float>(
      this->declare_parameter<double>("rc_manual.acc_xy", params.acc_xy));
    params.hold_max_xy = static_cast<float>(
      this->declare_parameter<double>("rc_manual.hold_max_xy", params.hold_max_xy));
    params.hold_max_z = static_cast<float>(
      this->declare_parameter<double>("rc_manual.hold_max_z", params.hold_max_z));
    params.yaw_rate_max_deg = static_cast<float>(
      this->declare_parameter<double>("rc_manual.yaw_rate_max_deg", params.yaw_rate_max_deg));
    params.expo =
      static_cast<float>(this->declare_parameter<double>("rc_manual.expo", params.expo));
    params.deadzone = static_cast<float>(
      this->declare_parameter<double>("rc_manual.deadzone", params.deadzone));
    params.v_xy_side_k = static_cast<float>(
      this->declare_parameter<double>("rc_manual.v_xy_side_k", params.v_xy_side_k));
    params.v_xy_back_k = static_cast<float>(
      this->declare_parameter<double>("rc_manual.v_xy_back_k", params.v_xy_back_k));
    params.v_xy_repos_min = static_cast<float>(
      this->declare_parameter<double>("rc_manual.v_xy_repos_min", params.v_xy_repos_min));
    params.stick_brake_eps = static_cast<float>(
      this->declare_parameter<double>("rc_manual.stick_brake_eps", params.stick_brake_eps));
    params.reset_jump_thresh = static_cast<float>(
      this->declare_parameter<double>("rc_manual.reset_jump_thresh", params.reset_jump_thresh));
    generator_.setParams(params);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RcManualTrajectoryModeNode::onTimer, this));
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
    const TrajectorySample sample = generator_.step(dt_s, state);

    offboard_control_mode_pub_->publish(makeOffboardControlMode(sample));
    trajectory_setpoint_pub_->publish(makeTrajectorySetpoint(sample));
  }

  px4_ros2::Context px4_context_;
  double publish_rate_hz_{50.0};
  bool initialized_{false};
  RcManualGenerator generator_{px4_context_};
  std::unique_ptr<TrajectoryStateProvider> state_provider_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RcManualTrajectoryModeNode>());
  rclcpp::shutdown();
  return 0;
}
