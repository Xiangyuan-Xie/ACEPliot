#include <am_position_ctbr_mode.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace
{
std::string readTextFile(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("Failed to open CTBR metadata file: " + path);
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
}  // namespace

CtbrDeploymentMetadata loadCtbrDeploymentMetadata(const std::string & metadata_path)
{
  const std::string json_text = readTextFile(metadata_path);

  CtbrDeploymentMetadata metadata;
  metadata.action_semantics = extractJsonString(json_text, "action_semantics");
  metadata.body_frame = extractJsonString(json_text, "body_frame");
  metadata.publish_frame = extractJsonString(json_text, "publish_frame");
  metadata.collective_preprocess = extractJsonString(json_text, "collective_preprocess");
  metadata.max_body_rate_rad_s = extractJsonFloat(json_text, "max_body_rate_rad_s");
  return metadata;
}

CtbrSetpointFrd mapRawCtbrActionToPx4Setpoint(
  const std::vector<float> & raw_action, float max_body_rate_rad_s, float collective_scale)
{
  const float roll_rate_flu =
    raw_action.size() > 0 ? std::tanh(raw_action[0]) * max_body_rate_rad_s : 0.0f;
  const float pitch_rate_flu =
    raw_action.size() > 1 ? std::tanh(raw_action[1]) * max_body_rate_rad_s : 0.0f;
  const float yaw_rate_flu =
    raw_action.size() > 2 ? std::tanh(raw_action[2]) * max_body_rate_rad_s : 0.0f;
  const float collective =
    raw_action.size() >
    3 ? (1.0f / (1.0f + std::exp(-2.0f * raw_action[3]))) * collective_scale : 0.0f;

  CtbrSetpointFrd setpoint;
  setpoint.body_rate_rad_s = Eigen::Vector3f(
    roll_rate_flu,
    -pitch_rate_flu,
    -yaw_rate_flu);
  setpoint.thrust = Eigen::Vector3f(
    0.0f,
    0.0f,
    -collective);
  return setpoint;
}

AmPositionCTBRMode::AmPositionCTBRMode(
  rclcpp::Node & node,
  const std::string & mode_name,
  bool activate_disarmed,
  const std::string & topic_namespace_prefix,
  const std::string & root_dir)
: AmPositionMotorMode(
    node,
    mode_name,
    activate_disarmed,
    topic_namespace_prefix,
    root_dir)
{
  const std::string model_path = node.get_parameter("model_path").as_string();
  const std::string default_metadata_path =
    model_path.size() >= 5 && model_path.substr(model_path.size() - 5) == ".onnx" ?
    model_path.substr(0, model_path.size() - 5) + ".json" :
    model_path + ".json";

  if (!node.has_parameter("metadata_path")) {
    node.declare_parameter("metadata_path", default_metadata_path);
  }
  metadata_path_ = node.get_parameter("metadata_path").as_string();
  metadata_ = loadCtbrDeploymentMetadata(metadata_path_);

  if (metadata_.action_semantics != "ctbr_raw" || metadata_.body_frame != "FLU" ||
    metadata_.publish_frame != "FRD" || metadata_.collective_preprocess != "sigmoid_2x")
  {
    throw std::runtime_error(
            "CTBR mode metadata mismatch in '" + metadata_path_ +
            "'. Expected action_semantics=ctbr_raw, body_frame=FLU, publish_frame=FRD, collective_preprocess=sigmoid_2x.");
  }

  if (!node.has_parameter("max_body_rate_rad_s")) {
    node.declare_parameter(
      "max_body_rate_rad_s",
      static_cast<double>(metadata_.max_body_rate_rad_s));
  }
  if (!node.has_parameter("ctbr_collective_scale")) {
    node.declare_parameter("ctbr_collective_scale", static_cast<double>(ctbr_collective_scale_));
  }

  max_body_rate_rad_s_ = static_cast<float>(node.get_parameter("max_body_rate_rad_s").as_double());
  ctbr_collective_scale_ =
    static_cast<float>(node.get_parameter("ctbr_collective_scale").as_double());

  // The deployed policy publishes PX4 rates setpoints, but its semantics are the training-side
  // raw CTBR action in FLU. We therefore keep a dedicated setpoint publisher and perform the
  // FLU->FRD conversion explicitly in applyAction().
  rates_setpoint_ = std::make_shared<px4_ros2::RatesSetpointType>(*this);
}

void AmPositionCTBRMode::applyAction(const TensorMap & action, float dt_s)
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

  const CtbrSetpointFrd setpoint =
    mapRawCtbrActionToPx4Setpoint(out_vec, max_body_rate_rad_s_, ctbr_collective_scale_);
  rates_setpoint_->update(setpoint.body_rate_rad_s, setpoint.thrust);
}
