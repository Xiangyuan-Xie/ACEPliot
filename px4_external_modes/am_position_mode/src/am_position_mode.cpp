/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <am_position_mode.hpp>

#include <math.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <rcl/time.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace am_position_mode
{
namespace
{
constexpr std::size_t kActionDim = 4;
constexpr std::size_t kCoreObservationDim = 16;
constexpr std::size_t kArmJointObservationDim = 4;
constexpr std::size_t kPolicyObservationDim =
  kCoreObservationDim + kArmJointObservationDim + kActionDim;
constexpr float kCommandZeroEpsilon = 1.0e-6F;
static_assert(kPolicyObservationDim == 24);

std::string readTextFile(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("Failed to open deployment metadata file: " + path);
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::string extractJsonString(const std::string & json_text, const std::string & key)
{
  const std::regex pattern("\"" + key + "\"\\s*:\\s*\"([^\"]+)\"");
  std::smatch match;
  if (!std::regex_search(json_text, match, pattern)) {
    throw std::runtime_error("Missing string metadata key '" + key + "'.");
  }
  return match[1].str();
}

float extractJsonFloat(const std::string & json_text, const std::string & key)
{
  const std::regex pattern("\"" + key + "\"\\s*:\\s*([-+0-9.eE]+)");
  std::smatch match;
  if (!std::regex_search(json_text, match, pattern)) {
    throw std::runtime_error("Missing numeric metadata key '" + key + "'.");
  }
  return std::stof(match[1].str());
}

std::string defaultMetadataPath(const std::string & model_path)
{
  constexpr char kOnnxExtension[] = ".onnx";
  if (model_path.size() >= sizeof(kOnnxExtension) - 1U &&
    model_path.compare(
      model_path.size() - (sizeof(kOnnxExtension) - 1U),
      sizeof(kOnnxExtension) - 1U,
      kOnnxExtension) == 0)
  {
    return model_path.substr(0, model_path.size() - (sizeof(kOnnxExtension) - 1U)) + ".json";
  }
  return model_path + ".json";
}

void validateFinitePositive(float value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0F) {
    throw std::invalid_argument(std::string(name) + " must be finite and greater than zero");
  }
}
}  // namespace

DeploymentMetadata loadDeploymentMetadata(const std::string & metadata_path)
{
  const std::string json_text = readTextFile(metadata_path);
  DeploymentMetadata metadata;
  metadata.action_semantics = extractJsonString(json_text, "action_semantics");
  metadata.body_frame = extractJsonString(json_text, "body_frame");
  metadata.publish_frame = extractJsonString(json_text, "publish_frame");
  metadata.collective_preprocess = extractJsonString(json_text, "collective_preprocess");
  metadata.max_body_rate_rad_s = extractJsonFloat(json_text, "max_body_rate_rad_s");
  validateFinitePositive(metadata.max_body_rate_rad_s, "max_body_rate_rad_s");
  if (metadata.action_semantics != "body_rate_thrust_raw" ||
    metadata.body_frame != "FLU" || metadata.publish_frame != "FRD" ||
    metadata.collective_preprocess != "sigmoid_2x")
  {
    throw std::runtime_error(
            "Deployment metadata must use action_semantics=body_rate_thrust_raw, "
            "body_frame=FLU, publish_frame=FRD, collective_preprocess=sigmoid_2x");
  }
  return metadata;
}

BodyRateThrustSetpointFrd mapRawActionToSetpoint(
  const std::vector<float> & raw_action,
  float max_body_rate_rad_s,
  float collective_scale)
{
  if (raw_action.size() != kActionDim) {
    throw std::invalid_argument("Body-rate/thrust action must contain exactly four values");
  }
  if (!std::all_of(
      raw_action.begin(), raw_action.end(),
      [](float value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("Body-rate/thrust action contains a non-finite value");
  }
  validateFinitePositive(max_body_rate_rad_s, "max_body_rate_rad_s");
  if (!std::isfinite(collective_scale) || collective_scale < 0.0F) {
    throw std::invalid_argument("collective_scale must be finite and non-negative");
  }

  const Eigen::Vector3f body_rate_flu(
    std::tanh(raw_action[0]) * max_body_rate_rad_s,
    std::tanh(raw_action[1]) * max_body_rate_rad_s,
    std::tanh(raw_action[2]) * max_body_rate_rad_s);
  const float collective =
    (1.0F / (1.0F + std::exp(-2.0F * raw_action[3]))) * collective_scale;

  BodyRateThrustSetpointFrd setpoint;
  setpoint.body_rate_rad_s = Eigen::Vector3f(
    body_rate_flu.x(), -body_rate_flu.y(), -body_rate_flu.z());
  setpoint.thrust = Eigen::Vector3f(0.0F, 0.0F, -collective);
  return setpoint;
}

AmPositionMode::AmPositionMode(
  rclcpp::Node & node,
  const std::string & mode_name,
  bool activate_disarmed,
  const std::string & topic_namespace_prefix,
  const std::string & root_dir)
: px4_ros2::ModeBase(
    node,
    px4_ros2::ModeBase::Settings(mode_name).activateEvenWhileDisarmed(activate_disarmed),
    topic_namespace_prefix),
  action_observation_buffer_(static_cast<int>(kActionDim), 1)
{
  modeRequirements().local_position = true;

  const std::string default_model_path = root_dir + "weights/policy.onnx";
  if (!node.has_parameter("model_path")) {
    node.declare_parameter("model_path", default_model_path);
  }
  const std::string model_path = node.get_parameter("model_path").as_string();
  policy_ = std::make_unique<policy_inference::OnnxPolicy>(model_path);
  if (!policy_->hasInput("obs")) {
    throw std::invalid_argument("AM Position policy must expose an input named 'obs'");
  }
  const auto & observation_shape = policy_->inputShape("obs");
  if (observation_shape.empty() ||
    (observation_shape.back() > 0 &&
    static_cast<std::size_t>(observation_shape.back()) != kPolicyObservationDim))
  {
    throw std::invalid_argument("AM Position policy 'obs' input must contain 24 values");
  }
  if (policy_->hasInput("h_in")) {
    recurrent_state_.configure(policy_->inputShape("h_in"));
  }

  if (!node.has_parameter("flight_log_compression")) {
    node.declare_parameter("flight_log_compression", true);
  }
  FlightLogger::RosbagParams logger_parameters;
  logger_parameters.root_dir = root_dir;
  logger_parameters.enable_compression =
    node.get_parameter("flight_log_compression").as_bool();
  logger_parameters.base_topic = "";
  try {
    flight_logger_ = std::make_unique<FlightLogger>(logger_parameters);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node.get_logger(), "FlightLogger initialization failed: %s", error.what());
  }

  robot_data_ = std::make_unique<ArmRobotData>(*this);
  configureClockSource(node);
  rates_setpoint_ = std::make_shared<px4_ros2::RatesSetpointType>(*this);
  action_observation_buffer_.insert(std::vector<float>(kActionDim, 0.0F));

  if (!node.has_parameter("metadata_path")) {
    node.declare_parameter("metadata_path", defaultMetadataPath(model_path));
  }
  metadata_path_ = node.get_parameter("metadata_path").as_string();
  metadata_ = loadDeploymentMetadata(metadata_path_);

  if (!node.has_parameter("max_body_rate_rad_s")) {
    node.declare_parameter(
      "max_body_rate_rad_s", static_cast<double>(metadata_.max_body_rate_rad_s));
  }
  if (!node.has_parameter("collective_scale")) {
    node.declare_parameter("collective_scale", static_cast<double>(collective_scale_));
  }
  max_body_rate_rad_s_ =
    static_cast<float>(node.get_parameter("max_body_rate_rad_s").as_double());
  collective_scale_ = static_cast<float>(node.get_parameter("collective_scale").as_double());
  validateFinitePositive(max_body_rate_rad_s_, "max_body_rate_rad_s");
  if (!std::isfinite(collective_scale_) || collective_scale_ < 0.0F) {
    throw std::invalid_argument("collective_scale must be finite and non-negative");
  }

  offboard_control_mode_topic_ +=
    px4_ros2::getMessageNameVersion<px4_msgs::msg::OffboardControlMode>();
  trajectory_setpoint_topic_ +=
    px4_ros2::getMessageNameVersion<px4_msgs::msg::TrajectorySetpoint>();
  if (!node.has_parameter("offboard_control_mode_topic")) {
    node.declare_parameter("offboard_control_mode_topic", offboard_control_mode_topic_);
  }
  if (!node.has_parameter("trajectory_setpoint_topic")) {
    node.declare_parameter("trajectory_setpoint_topic", trajectory_setpoint_topic_);
  }
  if (!node.has_parameter("offboard_setpoint_timeout_s")) {
    node.declare_parameter("offboard_setpoint_timeout_s", offboard_setpoint_timeout_s_);
  }
  offboard_control_mode_topic_ = node.get_parameter("offboard_control_mode_topic").as_string();
  trajectory_setpoint_topic_ = node.get_parameter("trajectory_setpoint_topic").as_string();
  offboard_setpoint_timeout_s_ =
    std::max(0.0, node.get_parameter("offboard_setpoint_timeout_s").as_double());

  offboard_control_mode_sub_ = node.create_subscription<px4_msgs::msg::OffboardControlMode>(
    offboard_control_mode_topic_, 10,
    std::bind(&AmPositionMode::offboardControlModeCallback, this, std::placeholders::_1));
  trajectory_setpoint_sub_ = node.create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    trajectory_setpoint_topic_, 10,
    std::bind(&AmPositionMode::trajectorySetpointCallback, this, std::placeholders::_1));
}

void AmPositionMode::onActivate()
{
  recurrent_state_.reset();
  action_observation_buffer_.reset();
  action_observation_buffer_.insert(std::vector<float>(kActionDim, 0.0F));
  has_offboard_control_mode_msg_ = false;
  has_trajectory_setpoint_msg_ = false;
  hover_lock_active_ = false;
  current_cmd_ref_ = CommandReference{};
  warned_waiting_for_sim_time_ = false;
}

void AmPositionMode::onDeactivate() {}

void AmPositionMode::updateSetpoint(float dt_s)
{
  robot_data_->updateState();
  logOdometryData();

  policy_inference::TensorMap inputs;
  getObservation(inputs, dt_s);
  if (inputs.empty()) {
    return;
  }
  logTensor("observations", inputs);

  const auto inference_start = std::chrono::steady_clock::now();
  const policy_inference::TensorMap actions = policy_->infer(inputs);
  const auto inference_end = std::chrono::steady_clock::now();
  if (flight_logger_ && flight_logger_->isOpen()) {
    std_msgs::msg::Float32 latency;
    latency.data = static_cast<float>(
      std::chrono::duration<double, std::milli>(inference_end - inference_start).count());
    rclcpp::Time now;
    if (getCurrentModeTime(now)) {
      flight_logger_->log("inference_time_ms", latency, now.seconds());
    }
  }

  logTensor("actions", actions);
  applyAction(actions);
}

void AmPositionMode::getObservation(policy_inference::TensorMap & inputs, float dt_s)
{
  updateTargets(dt_s);
  logTargetValues(dt_s);

  const Eigen::Vector3f & root_pos_w = robot_data_->RootPosW();
  const Eigen::Quaternionf & root_quat_w = robot_data_->RootQuatW();
  const Eigen::Vector3f & projected_gravity = robot_data_->ProjectedGravityB();
  const Eigen::Vector3f & linear_velocity_b = robot_data_->RootLinVelB();
  const Eigen::Vector3f & angular_velocity_b = robot_data_->RootAngVelB();

  Eigen::Vector3f position_error_b = Eigen::Vector3f::Zero();
  const auto [desired_position_b, unused_rotation] = subtract_frame_transforms(
    root_pos_w, root_quat_w, current_cmd_ref_.desired_pos_w);
  (void)unused_rotation;
  if (desired_position_b.allFinite()) {
    position_error_b = desired_position_b;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (current_cmd_ref_.lin_cmd_active[axis]) {
      position_error_b[static_cast<Eigen::Index>(axis)] = 0.0F;
    }
  }

  const float desired_yaw_w = std::get<2>(euler_xyz_from_quat(current_cmd_ref_.desired_quat_w));
  float yaw_error = wrapToPi(desired_yaw_w - robot_data_->HeadingW());
  if (current_cmd_ref_.ang_cmd_active[2]) {
    yaw_error = 0.0F;
  }
  const float cos_yaw = std::cos(yaw_error);
  const float sin_yaw = std::sin(yaw_error);
  const Eigen::Vector3f linear_velocity_error_b =
    current_cmd_ref_.desired_lin_vel_b - linear_velocity_b;
  const Eigen::Vector3f angular_velocity_error_b =
    current_cmd_ref_.desired_ang_vel_b - angular_velocity_b;

  std::vector<float> observation;
  observation.reserve(kPolicyObservationDim);
  observation.insert(
    observation.end(), position_error_b.data(), position_error_b.data() + position_error_b.size());
  observation.insert(observation.end(), {cos_yaw, -sin_yaw, sin_yaw, cos_yaw});
  observation.insert(
    observation.end(), projected_gravity.data(),
    projected_gravity.data() + projected_gravity.size());
  observation.insert(
    observation.end(), linear_velocity_error_b.data(),
    linear_velocity_error_b.data() + linear_velocity_error_b.size());
  observation.insert(
    observation.end(), angular_velocity_error_b.data(),
    angular_velocity_error_b.data() + angular_velocity_error_b.size());

  const std::vector<float> & arm_position = robot_data_->ArmPosition();
  if (arm_position.empty()) {
    observation.insert(observation.end(), kArmJointObservationDim, 0.0F);
  } else if (arm_position.size() == kArmJointObservationDim) {
    observation.insert(observation.end(), arm_position.begin(), arm_position.end());
  } else {
    RCLCPP_ERROR(
      node().get_logger(), "Arm state has %zu joints; expected %zu. Skipping inference.",
      arm_position.size(), kArmJointObservationDim);
    return;
  }

  const std::vector<float> action_history = action_observation_buffer_.get_flattened_history();
  if (action_history.size() != kActionDim) {
    RCLCPP_ERROR(
      node().get_logger(), "Action history has %zu values; expected %zu. Skipping inference.",
      action_history.size(), kActionDim);
    return;
  }
  observation.insert(observation.end(), action_history.begin(), action_history.end());
  if (observation.size() != kPolicyObservationDim) {
    throw std::logic_error("AM Position observation assembly produced an invalid size");
  }

  inputs.emplace("obs", std::move(observation));
  recurrent_state_.appendInput(inputs);
}

void AmPositionMode::applyAction(const policy_inference::TensorMap & action)
{
  const auto output = action.find("actions");
  if (output == action.end()) {
    throw std::invalid_argument("AM Position policy did not produce 'actions'");
  }
  const BodyRateThrustSetpointFrd setpoint = mapRawActionToSetpoint(
    output->second, max_body_rate_rad_s_, collective_scale_);

  action_observation_buffer_.insert(output->second);
  recurrent_state_.updateFromOutput(action);
  rates_setpoint_->update(setpoint.body_rate_rad_s, setpoint.thrust);
}

void AmPositionMode::updateTargets(float dt_s)
{
  (void)dt_s;
  const Eigen::Vector3f & root_pos_w = robot_data_->RootPosW();
  const Eigen::Quaternionf & root_quat_w = robot_data_->RootQuatW();
  const float root_yaw_w = robot_data_->HeadingW();

  Eigen::Vector3f desired_linear_velocity_b = Eigen::Vector3f::Zero();
  Eigen::Vector3f desired_angular_velocity_b = Eigen::Vector3f::Zero();
  std::array<bool, 3> linear_active{{false, false, false}};
  std::array<bool, 3> position_active{{false, false, false}};
  std::array<bool, 3> angular_active{{false, false, false}};

  if (!hover_lock_active_) {
    hover_lock_active_ = true;
    hover_lock_pos_w_ = root_pos_w;
    hover_lock_yaw_w_ = root_yaw_w;
  }

  if (hasFreshExternalReference()) {
    const AmPositionOffboardReference reference = buildAmPositionOffboardReference(
      last_offboard_control_mode_msg_, last_trajectory_setpoint_msg_, hover_lock_pos_w_,
      hover_lock_yaw_w_, root_quat_w);
    if (reference.valid) {
      desired_linear_velocity_b = reference.desired_lin_vel_b;
      desired_angular_velocity_b = reference.desired_ang_vel_b;
      position_active = reference.position_active;
      linear_active = reference.velocity_active;
      angular_active[2] =
        reference.has_yaw_rate_cmd &&
        std::abs(desired_angular_velocity_b.z()) > kCommandZeroEpsilon;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        if (position_active[axis]) {
          hover_lock_pos_w_[static_cast<Eigen::Index>(axis)] =
            reference.desired_pos_w[static_cast<Eigen::Index>(axis)];
        }
      }
      if (std::isfinite(last_trajectory_setpoint_msg_.yaw)) {
        hover_lock_yaw_w_ = px4_ros2::yawNedToEnu(last_trajectory_setpoint_msg_.yaw);
      }
    }
  }

  const auto [relative_lock_position_b, unused_rotation] = subtract_frame_transforms(
    root_pos_w, root_quat_w, hover_lock_pos_w_);
  (void)unused_rotation;
  Eigen::Vector3f lock_position_b = relative_lock_position_b;
  if (lock_position_b.allFinite()) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      if (linear_active[axis] && !position_active[axis]) {
        lock_position_b[static_cast<Eigen::Index>(axis)] = 0.0F;
      }
    }
    hover_lock_pos_w_ = root_pos_w + root_quat_w * lock_position_b;
  } else {
    hover_lock_pos_w_ = root_pos_w;
  }

  float yaw_error = wrapToPi(hover_lock_yaw_w_ - root_yaw_w);
  if (angular_active[2]) {
    yaw_error = 0.0F;
  }
  hover_lock_yaw_w_ = wrapToPi(root_yaw_w + yaw_error);

  current_cmd_ref_.desired_lin_vel_b = desired_linear_velocity_b;
  current_cmd_ref_.desired_ang_vel_b = desired_angular_velocity_b;
  current_cmd_ref_.desired_pos_w = hover_lock_pos_w_;
  current_cmd_ref_.desired_quat_w = yawOnlyQuat(hover_lock_yaw_w_);
  current_cmd_ref_.lin_cmd_active = linear_active;
  current_cmd_ref_.ang_cmd_active = angular_active;
  current_cmd_ref_.has_lin_vel_cmd = anyAxisActive(linear_active);
  current_cmd_ref_.has_ang_vel_cmd = anyAxisActive(angular_active);
}

void AmPositionMode::logTargetValues(float dt_s)
{
  (void)dt_s;
  if (!flight_logger_ || !flight_logger_->isOpen()) {
    return;
  }
  rclcpp::Time now;
  if (!getCurrentModeTime(now)) {
    return;
  }

  std_msgs::msg::Float32MultiArray target_twist;
  target_twist.data = {
    current_cmd_ref_.desired_lin_vel_b.x(), current_cmd_ref_.desired_lin_vel_b.y(),
    current_cmd_ref_.desired_lin_vel_b.z(), current_cmd_ref_.desired_ang_vel_b.x(),
    current_cmd_ref_.desired_ang_vel_b.y(), current_cmd_ref_.desired_ang_vel_b.z()};
  flight_logger_->log("target_twist_cmd_b", target_twist, now.seconds());

  std_msgs::msg::Float32MultiArray target_pose;
  target_pose.data = {
    current_cmd_ref_.desired_pos_w.x(), current_cmd_ref_.desired_pos_w.y(),
    current_cmd_ref_.desired_pos_w.z(), current_cmd_ref_.desired_quat_w.w(),
    current_cmd_ref_.desired_quat_w.x(), current_cmd_ref_.desired_quat_w.y(),
    current_cmd_ref_.desired_quat_w.z()};
  flight_logger_->log("target_pose_ref_w", target_pose, now.seconds());

  std_msgs::msg::Float32MultiArray target_metadata;
  target_metadata.data = {
    current_cmd_ref_.has_lin_vel_cmd ? 1.0F : 0.0F,
    current_cmd_ref_.has_ang_vel_cmd ? 1.0F : 0.0F};
  flight_logger_->log("target_reference_meta", target_metadata, now.seconds());
}

void AmPositionMode::logOdometryData()
{
  if (!flight_logger_ || !flight_logger_->isOpen() || !robot_data_) {
    return;
  }
  rclcpp::Time now;
  if (!getCurrentModeTime(now)) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = now;
  pose.header.frame_id = "world";
  const Eigen::Vector3f & position = robot_data_->RootPosW();
  pose.pose.position.x = position.x();
  pose.pose.position.y = position.y();
  pose.pose.position.z = position.z();
  const Eigen::Quaternionf & orientation = robot_data_->RootQuatW();
  pose.pose.orientation.w = orientation.w();
  pose.pose.orientation.x = orientation.x();
  pose.pose.orientation.y = orientation.y();
  pose.pose.orientation.z = orientation.z();
  flight_logger_->log("pose", pose, now.seconds());

  geometry_msgs::msg::TwistStamped twist;
  twist.header.stamp = now;
  twist.header.frame_id = "base_link";
  const Eigen::Vector3f & linear_velocity = robot_data_->RootLinVelB();
  const Eigen::Vector3f & angular_velocity = robot_data_->RootAngVelB();
  twist.twist.linear.x = linear_velocity.x();
  twist.twist.linear.y = linear_velocity.y();
  twist.twist.linear.z = linear_velocity.z();
  twist.twist.angular.x = angular_velocity.x();
  twist.twist.angular.y = angular_velocity.y();
  twist.twist.angular.z = angular_velocity.z();
  flight_logger_->log("twist", twist, now.seconds());

  tf2_msgs::msg::TFMessage transform_message;
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now;
  transform.header.frame_id = "world";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = position.x();
  transform.transform.translation.y = position.y();
  transform.transform.translation.z = position.z();
  transform.transform.rotation = pose.pose.orientation;
  transform_message.transforms.emplace_back(transform);
  flight_logger_->log("tf", transform_message, now.seconds());
}

void AmPositionMode::logTensor(
  const std::string & topic,
  const policy_inference::TensorMap & tensors)
{
  if (!flight_logger_ || !flight_logger_->isOpen()) {
    return;
  }
  rclcpp::Time now;
  if (!getCurrentModeTime(now)) {
    return;
  }
  for (const auto & [name, values] : tensors) {
    std_msgs::msg::Float32MultiArray message;
    message.data.assign(values.begin(), values.end());
    flight_logger_->log(topic + "/" + name, message, now.seconds());
  }
}

void AmPositionMode::offboardControlModeCallback(
  const px4_msgs::msg::OffboardControlMode::SharedPtr message)
{
  last_offboard_control_mode_msg_ = *message;
  has_offboard_control_mode_msg_ =
    updateExternalReferenceReceiptTime(last_offboard_control_mode_time_);
}

void AmPositionMode::trajectorySetpointCallback(
  const px4_msgs::msg::TrajectorySetpoint::SharedPtr message)
{
  last_trajectory_setpoint_msg_ = *message;
  has_trajectory_setpoint_msg_ =
    updateExternalReferenceReceiptTime(last_trajectory_setpoint_time_);
}

bool AmPositionMode::hasFreshExternalReference()
{
  if (!has_offboard_control_mode_msg_ || !has_trajectory_setpoint_msg_) {
    return false;
  }
  rclcpp::Time now;
  if (!getCurrentModeTime(now)) {
    return false;
  }
  return (now - last_offboard_control_mode_time_).seconds() <= offboard_setpoint_timeout_s_ &&
         (now - last_trajectory_setpoint_time_).seconds() <= offboard_setpoint_timeout_s_;
}

void AmPositionMode::simClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr message)
{
  latest_sim_time_ = rclcpp::Time(message->clock, RCL_ROS_TIME);
  has_sim_time_ = true;
  warned_waiting_for_sim_time_ = false;

  const auto clock = node().get_clock();
  std::lock_guard<std::mutex> clock_lock(clock->get_clock_mutex());
  auto * clock_handle = clock->get_clock_handle();
  if (!clock->ros_time_is_active()) {
    const rcl_ret_t enable_result = rcl_enable_ros_time_override(clock_handle);
    if (enable_result != RCL_RET_OK) {
      RCLCPP_ERROR(
        node().get_logger(), "Failed to enable ROS time override from '%s': %s",
        sim_clock_topic_.c_str(), rcl_get_error_string().str);
      rcl_reset_error();
      return;
    }
  }
  const rcl_ret_t update_result =
    rcl_set_ros_time_override(clock_handle, latest_sim_time_.nanoseconds());
  if (update_result != RCL_RET_OK) {
    RCLCPP_ERROR(
      node().get_logger(), "Failed to update ROS time from '%s': %s",
      sim_clock_topic_.c_str(), rcl_get_error_string().str);
    rcl_reset_error();
  }
}

bool AmPositionMode::getCurrentModeTime(rclcpp::Time & now)
{
  if (!use_sim_time_) {
    now = node().get_clock()->now();
    return true;
  }
  if (!has_sim_time_) {
    return false;
  }
  now = latest_sim_time_;
  return true;
}

void AmPositionMode::configureClockSource(rclcpp::Node & node)
{
  node.get_parameter("use_sim_time", use_sim_time_);
  if (!node.has_parameter("sim_clock_topic")) {
    node.declare_parameter("sim_clock_topic", sim_clock_topic_);
  }
  sim_clock_topic_ = node.get_parameter("sim_clock_topic").as_string();
  if (use_sim_time_) {
    sim_clock_sub_ = node.create_subscription<rosgraph_msgs::msg::Clock>(
      sim_clock_topic_, rclcpp::ClockQoS(),
      std::bind(&AmPositionMode::simClockCallback, this, std::placeholders::_1));
  }
}

bool AmPositionMode::updateExternalReferenceReceiptTime(rclcpp::Time & receipt_time)
{
  if (getCurrentModeTime(receipt_time)) {
    return true;
  }
  if (use_sim_time_ && !warned_waiting_for_sim_time_) {
    RCLCPP_WARN(
      node().get_logger(), "Ignoring Offboard references until '%s' publishes clock data.",
      sim_clock_topic_.c_str());
    warned_waiting_for_sim_time_ = true;
  }
  return false;
}

}  // namespace am_position_mode
