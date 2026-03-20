#include <rl_mc_arm_position_rates_thrust_mode.hpp>
#include <algorithm>

RlMCArmPositionRatesThrustMode::RlMCArmPositionRatesThrustMode(
  rclcpp::Node & node,
  const std::string & mode_name,
  bool activate_disarmed,
  const std::string & topic_namespace_prefix,
  const std::string & root_dir)
: RlMCArmPositionDirectActuatorsMode(
    node,
    mode_name,
    activate_disarmed,
    topic_namespace_prefix,
    root_dir)
{
  // Declare optional scaling parameters once so launch files can override them.
  if (!node.has_parameter("max_body_rate_rad_s")) {
    node.declare_parameter("max_body_rate_rad_s", static_cast<double>(max_body_rate_rad_s_));
  }
  if (!node.has_parameter("max_collective_thrust")) {
    node.declare_parameter("max_collective_thrust", static_cast<double>(max_collective_thrust_));
  }

  max_body_rate_rad_s_ = static_cast<float>(node.get_parameter("max_body_rate_rad_s").as_double());
  max_collective_thrust_ =
    static_cast<float>(node.get_parameter("max_collective_thrust").as_double());

  // Use rates setpoint output instead of direct actuator channels for this variant.
  rates_setpoint_ = std::make_shared<px4_ros2::RatesSetpointType>(*this);
}

void RlMCArmPositionRatesThrustMode::applyAction(const TensorMap & action, float dt_s)
{
  (void)dt_s;

  // Validate model output and exit early when action tensor is unavailable.
  auto it = action.find("actions");
  if (it == action.end() || !std::holds_alternative<std::vector<float>>(it->second)) {
    return;
  }

  // Keep action history and recurrent state behavior consistent with base mode.
  const auto & out_vec = std::get<std::vector<float>>(it->second);
  getActionObsBuffer().insert(out_vec);
  rnnState().updateFromOutput(action);

  // Convert normalized policy outputs to physical rates and thrust commands.
  const float roll_rate_norm = out_vec.size() > 0 ? std::clamp(out_vec[0], -1.0f, 1.0f) : 0.0f;
  const float pitch_rate_norm = out_vec.size() > 1 ? std::clamp(out_vec[1], -1.0f, 1.0f) : 0.0f;
  const float yaw_rate_norm = out_vec.size() > 2 ? std::clamp(out_vec[2], -1.0f, 1.0f) : 0.0f;
  const float collective_norm = out_vec.size() > 3 ? std::clamp(out_vec[3], 0.0f, 1.0f) : 0.0f;

  Eigen::Vector3f rate_setpoint_frd_rad(
    roll_rate_norm * max_body_rate_rad_s_,
    pitch_rate_norm * max_body_rate_rad_s_,
    yaw_rate_norm * max_body_rate_rad_s_);

  Eigen::Vector3f thrust_setpoint_frd(
    0.0f,
    0.0f,
    -collective_norm * max_collective_thrust_);

  // Publish FRD angular-rate and thrust setpoints to PX4.
  rates_setpoint_->update(rate_setpoint_frd_rad, thrust_setpoint_frd);
}
