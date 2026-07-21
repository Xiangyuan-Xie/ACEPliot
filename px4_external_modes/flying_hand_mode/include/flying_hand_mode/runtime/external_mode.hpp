#pragma once

#include <flying_hand_mode/runtime/controller_types.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <functional>
#include <memory>
#include <string>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace flying_hand_mode::runtime
{

struct FlyingHandModeConfiguration
{
  std::string mode_name;
  std::string diagnostic_name;
  std::string hardware_id;
  std::string topic_prefix;
  bool require_calibration_confirmation{false};
  bool monitor_control_allocator{false};
};

using EndEffectorPoseCallback = std::function<Eigen::Isometry3d(const JointVector &)>;

enum class FaultReason
{
  kNone,
  kShadowOnly,
  kCalibrationUnconfirmed,
  kControllerUnavailable,
  kControllerOutputMissing,
  kControllerOutputStale,
  kVehicleStatusMissing,
  kNotArmed,
  kUnsupportedVehicle,
  kVehicleFailsafe,
  kLandStateMissing,
  kNotAirborne,
  kOdometryInvalid,
  kOdometryStale,
  kImuInvalid,
  kImuStale,
  kArmStateInvalid,
  kArmStateStale,
  kFollowerNotReady,
  kAllocatorStatusMissing,
  kAllocatorUnachieved,
  kAllocatorSaturated,
  kExternalCommandPublisher,
  kSolverTimeout,
  kSolverInvalid,
  kSetpointRejected,
};

const char * faultReasonName(FaultReason reason) noexcept;

struct SafetySnapshot
{
  bool closed_loop{false};
  bool calibration_confirmed{true};
  bool controller_available{false};
  bool controller_output_ready{false};
  bool vehicle_status_received{false};
  bool armed{false};
  bool rotary_wing{false};
  bool vehicle_failsafe{false};
  bool land_state_received{false};
  bool airborne{false};
  bool odometry_valid{false};
  bool imu_valid{false};
  bool arm_state_valid{false};
  bool follower_ready{false};
  bool allocator_status_required{false};
  bool allocator_status_received{false};
  bool allocator_setpoint_achieved{true};
  bool allocator_saturated{false};
  bool external_command_publisher{false};
  double odometry_age_s{0.0};
  double imu_age_s{0.0};
  double arm_state_age_s{0.0};
  double allocator_status_age_s{0.0};
  double controller_output_age_s{0.0};
};

class FlyingHandSafety
{
public:
  FlyingHandSafety(double maximum_state_age_s, double solver_budget_s, int timeout_limit);

  FaultReason readiness(const SafetySnapshot & snapshot) const noexcept;
  bool recordSolverResult(double elapsed_s, bool valid) noexcept;
  void latch(FaultReason reason) noexcept;
  void reset() noexcept;

  bool faultLatched() const noexcept;
  FaultReason faultReason() const noexcept;
  int consecutiveTimeouts() const noexcept;

private:
  double maximum_state_age_s_;
  double solver_budget_s_;
  int timeout_limit_;
  int consecutive_timeouts_{0};
  FaultReason fault_reason_{FaultReason::kNone};
};

class FlyingHandMode : public px4_ros2::ModeBase
{
public:
  explicit FlyingHandMode(
    rclcpp::Node & node, FlyingHandModeConfiguration configuration,
    EndEffectorPoseCallback end_effector_pose_flu,
    const std::string & px4_topic_namespace_prefix = "",
    ControllerCallbacks controller = {});
  ~FlyingHandMode() override;

  void setFallbackCallback(std::function<void(FaultReason)> callback);

  void checkArmingAndRunConditions(
    px4_ros2::HealthAndArmingCheckReporter & reporter) override;
  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float dt_s) override;
  void prepareShutdown() noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

class FlyingHandModeExecutor : public px4_ros2::ModeExecutorBase
{
public:
  explicit FlyingHandModeExecutor(
    FlyingHandMode & mode, const std::string & px4_topic_namespace_prefix = "");
  ~FlyingHandModeExecutor() override;

  void onActivate() override;
  void onDeactivate(DeactivateReason reason) override;

private:
  void requestPositionFallback(FaultReason reason);

  class Impl;
  FlyingHandMode & mode_;
  std::unique_ptr<Impl> impl_;
};

}  // namespace flying_hand_mode::runtime
