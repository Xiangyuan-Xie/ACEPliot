#pragma once

#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <am_position_motor_mode.hpp>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

/// @brief Default mode name for the CTBR arm-position variant.
static constexpr char kAmPositionCTBRModeName[] = "AM Position CTBR";

struct CtbrDeploymentMetadata
{
  std::string action_semantics;
  std::string body_frame;
  std::string publish_frame;
  std::string collective_preprocess;
  float max_body_rate_rad_s{6.0f};
};

struct CtbrSetpointFrd
{
  Eigen::Vector3f body_rate_rad_s{Eigen::Vector3f::Zero()};
  Eigen::Vector3f thrust{Eigen::Vector3f::Zero()};
};

CtbrDeploymentMetadata loadCtbrDeploymentMetadata(const std::string & metadata_path);
CtbrSetpointFrd mapRawCtbrActionToPx4Setpoint(
  const std::vector<float> & raw_action, float max_body_rate_rad_s, float collective_scale);

/**
 * @class AmPositionCTBRMode
 * @brief Arm-position variant that outputs CTBR actions aligned with training semantics.
 *
 * This class reuses observation and target logic from
 * AmPositionMotorMode and only overrides action application.
 */
class AmPositionCTBRMode : public AmPositionMotorMode
{
public:
  /**
   * @brief Construct a CTBR arm-position mode instance.
   * @param node ROS2 node handle.
   * @param mode_name PX4 external mode display name.
   * @param activate_disarmed Whether this mode can activate while disarmed.
   * @param topic_namespace_prefix PX4 topic namespace prefix.
   * @param root_dir Mode root directory for resolving resources.
   */
  explicit AmPositionCTBRMode(
    rclcpp::Node & node,
    const std::string & mode_name = kAmPositionCTBRModeName,
    bool activate_disarmed = kAmPositionActivateEvenWhileDisarmed,
    const std::string & topic_namespace_prefix = "",
    const std::string & root_dir = ROOT_DIR);

  /// @brief Default destructor.
  ~AmPositionCTBRMode() override = default;

  /// @brief Applies policy outputs as CTBR actions and publishes FRD PX4 setpoints.
  void applyAction(const TensorMap & action, float dt_s) override;

private:
  /// @brief Rates+thrust output interface.
  std::shared_ptr<px4_ros2::RatesSetpointType> rates_setpoint_;
  /// @brief Metadata path associated with the exported ONNX model.
  std::string metadata_path_;
  /// @brief Loaded CTBR deployment metadata used to validate model semantics.
  CtbrDeploymentMetadata metadata_;
  /// @brief Max absolute body-rate command used to scale raw CTBR outputs.
  float max_body_rate_rad_s_{6.0f};
  /// @brief Optional post-sigmoid collective scale. Defaults to the training-side value of 1.0.
  float ctbr_collective_scale_{1.0f};
};
