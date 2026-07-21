#include <flying_hand_mode/runtime/controller_worker.hpp>
#include <flying_hand_mode/runtime/external_mode.hpp>
#include <flying_hand_mode/runtime/topic_utils.hpp>
#include <flying_hand_mode/runtime/wrench_setpoint_type.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <px4_msgs/msg/control_allocator_status.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_ros2/components/events.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace flying_hand_mode::runtime
{
namespace
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;
constexpr int kAllocatorSaturationLimit = 5;

template<typename MessageT>
std::string px4Topic(const std::string & prefix, const std::string & configured_topic)
{
  const std::string base = joinTopicPrefix(prefix, configured_topic);
  return base + px4_ros2::getMessageNameVersion<MessageT>();
}

double ageSeconds(const SteadyTime & sample_time, const SteadyTime & now)
{
  if (sample_time == SteadyTime{}) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(now - sample_time).count();
}

struct ControllerSampleTiming
{
  std::uint64_t timestamp_us{0};
  SteadyTime oldest_receipt_time{};
  SteadyTime odometry_receipt_time{};
  double odometry_source_age_at_receipt_s{0.0};

  bool stale(const SteadyTime & now, double maximum_age_s) const noexcept
  {
    return timestamp_us == 0 || !std::isfinite(odometry_source_age_at_receipt_s) ||
           odometry_source_age_at_receipt_s < 0.0 ||
           ageSeconds(oldest_receipt_time, now) > maximum_age_s ||
           odometry_source_age_at_receipt_s + ageSeconds(odometry_receipt_time, now) >
           maximum_age_s;
  }
};

bool finiteQuaternion(const Eigen::Quaterniond & quaternion)
{
  return quaternion.coeffs().allFinite() && quaternion.squaredNorm() > 1.0e-12;
}

geometry_msgs::msg::WrenchStamped makeWrenchMessage(
  const rclcpp::Time & stamp, const PhysicalWrench & wrench)
{
  geometry_msgs::msg::WrenchStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = "base_link_flu";
  message.wrench.force.x = wrench.force_b_n.x();
  message.wrench.force.y = wrench.force_b_n.y();
  message.wrench.force.z = wrench.force_b_n.z();
  message.wrench.torque.x = wrench.moment_b_nm.x();
  message.wrench.torque.y = wrench.moment_b_nm.y();
  message.wrench.torque.z = wrench.moment_b_nm.z();
  return message;
}

}  // namespace

bool ControllerOutput::allFinite() const noexcept
{
  return nominal_wrench_flu.allFinite() && adaptive_wrench_flu.allFinite() &&
         applied_wrench_flu.allFinite() && normalized_thrust_frd.allFinite() &&
         normalized_torque_frd.allFinite() && arm_position_command_rad.allFinite() &&
         std::isfinite(tracking_cost) && std::isfinite(allocation_condition_number);
}

bool ControllerOutput::normalizedCommandValid() const noexcept
{
  constexpr float kMinimumUpwardThrust = 1.0e-3F;
  return allFinite() && (normalized_thrust_frd.array().abs() <= 1.0F).all() &&
         normalized_thrust_frd.z() <= -kMinimumUpwardThrust &&
         (normalized_torque_frd.array().abs() <= 1.0F).all();
}

const char * faultReasonName(FaultReason reason) noexcept
{
  switch (reason) {
    case FaultReason::kNone: return "none";
    case FaultReason::kShadowOnly: return "shadow_only";
    case FaultReason::kCalibrationUnconfirmed: return "calibration_unconfirmed";
    case FaultReason::kControllerUnavailable: return "controller_unavailable";
    case FaultReason::kControllerOutputMissing: return "controller_output_missing";
    case FaultReason::kControllerOutputStale: return "controller_output_stale";
    case FaultReason::kVehicleStatusMissing: return "vehicle_status_missing";
    case FaultReason::kNotArmed: return "not_armed";
    case FaultReason::kUnsupportedVehicle: return "unsupported_vehicle";
    case FaultReason::kVehicleFailsafe: return "vehicle_failsafe";
    case FaultReason::kLandStateMissing: return "land_state_missing";
    case FaultReason::kNotAirborne: return "not_airborne";
    case FaultReason::kOdometryInvalid: return "odometry_invalid";
    case FaultReason::kOdometryStale: return "odometry_stale";
    case FaultReason::kImuInvalid: return "imu_invalid";
    case FaultReason::kImuStale: return "imu_stale";
    case FaultReason::kArmStateInvalid: return "arm_state_invalid";
    case FaultReason::kArmStateStale: return "arm_state_stale";
    case FaultReason::kFollowerNotReady: return "follower_not_ready";
    case FaultReason::kAllocatorStatusMissing: return "allocator_status_missing";
    case FaultReason::kAllocatorUnachieved: return "allocator_unachieved";
    case FaultReason::kAllocatorSaturated: return "allocator_saturated";
    case FaultReason::kExternalCommandPublisher: return "external_command_publisher";
    case FaultReason::kSolverTimeout: return "solver_timeout";
    case FaultReason::kSolverInvalid: return "solver_invalid";
    case FaultReason::kSetpointRejected: return "setpoint_rejected";
  }
  return "unknown";
}

FlyingHandSafety::FlyingHandSafety(
  double maximum_state_age_s, double solver_budget_s, int timeout_limit)
: maximum_state_age_s_(maximum_state_age_s),
  solver_budget_s_(solver_budget_s),
  timeout_limit_(timeout_limit)
{
  if (!std::isfinite(maximum_state_age_s_) || maximum_state_age_s_ <= 0.0 ||
    !std::isfinite(solver_budget_s_) || solver_budget_s_ <= 0.0 || timeout_limit_ <= 0)
  {
    throw std::invalid_argument("Flying Hand safety limits must be positive and finite");
  }
}

FaultReason FlyingHandSafety::readiness(const SafetySnapshot & snapshot) const noexcept
{
  if (!snapshot.closed_loop) {
    return FaultReason::kShadowOnly;
  }
  if (!snapshot.calibration_confirmed) {
    return FaultReason::kCalibrationUnconfirmed;
  }
  if (!snapshot.controller_available) {
    return FaultReason::kControllerUnavailable;
  }
  if (!snapshot.controller_output_ready) {
    return FaultReason::kControllerOutputMissing;
  }
  if (snapshot.controller_output_age_s > maximum_state_age_s_) {
    return FaultReason::kControllerOutputStale;
  }
  if (!snapshot.vehicle_status_received) {
    return FaultReason::kVehicleStatusMissing;
  }
  if (!snapshot.armed) {
    return FaultReason::kNotArmed;
  }
  if (!snapshot.rotary_wing) {
    return FaultReason::kUnsupportedVehicle;
  }
  if (snapshot.vehicle_failsafe) {
    return FaultReason::kVehicleFailsafe;
  }
  if (!snapshot.land_state_received) {
    return FaultReason::kLandStateMissing;
  }
  if (!snapshot.airborne) {
    return FaultReason::kNotAirborne;
  }
  if (!snapshot.odometry_valid) {
    return FaultReason::kOdometryInvalid;
  }
  if (snapshot.odometry_age_s > maximum_state_age_s_) {
    return FaultReason::kOdometryStale;
  }
  if (!snapshot.imu_valid) {
    return FaultReason::kImuInvalid;
  }
  if (snapshot.imu_age_s > maximum_state_age_s_) {
    return FaultReason::kImuStale;
  }
  if (!snapshot.arm_state_valid) {
    return FaultReason::kArmStateInvalid;
  }
  if (snapshot.arm_state_age_s > maximum_state_age_s_) {
    return FaultReason::kArmStateStale;
  }
  if (!snapshot.follower_ready) {
    return FaultReason::kFollowerNotReady;
  }
  if (snapshot.allocator_status_required) {
    if (!snapshot.allocator_status_received) {
      return FaultReason::kAllocatorStatusMissing;
    }
    if (!snapshot.allocator_setpoint_achieved) {
      return FaultReason::kAllocatorUnachieved;
    }
    if (snapshot.allocator_saturated) {
      return FaultReason::kAllocatorSaturated;
    }
  }
  if (snapshot.external_command_publisher) {
    return FaultReason::kExternalCommandPublisher;
  }
  return FaultReason::kNone;
}

bool FlyingHandSafety::recordSolverResult(double elapsed_s, bool valid) noexcept
{
  if (faultLatched()) {
    return false;
  }
  if (!valid || !std::isfinite(elapsed_s) || elapsed_s < 0.0) {
    latch(FaultReason::kSolverInvalid);
    return false;
  }
  if (elapsed_s > solver_budget_s_) {
    ++consecutive_timeouts_;
    if (consecutive_timeouts_ >= timeout_limit_) {
      latch(FaultReason::kSolverTimeout);
    }
    return false;
  }
  consecutive_timeouts_ = 0;
  return true;
}

void FlyingHandSafety::latch(FaultReason reason) noexcept
{
  if (reason != FaultReason::kNone && fault_reason_ == FaultReason::kNone) {
    fault_reason_ = reason;
  }
}

void FlyingHandSafety::reset() noexcept
{
  consecutive_timeouts_ = 0;
  fault_reason_ = FaultReason::kNone;
}

bool FlyingHandSafety::faultLatched() const noexcept
{
  return fault_reason_ != FaultReason::kNone;
}

FaultReason FlyingHandSafety::faultReason() const noexcept
{
  return fault_reason_;
}

int FlyingHandSafety::consecutiveTimeouts() const noexcept
{
  return consecutive_timeouts_;
}

class FlyingHandMode::Impl
{
public:
  Impl(
    FlyingHandMode & mode, rclcpp::Node & node,
    FlyingHandModeConfiguration configuration,
    EndEffectorPoseCallback end_effector_pose_flu,
    const std::string & px4_prefix, ControllerCallbacks controller)
  : mode_(mode),
    node_(node),
    configuration_(std::move(configuration)),
    end_effector_pose_flu_(std::move(end_effector_pose_flu)),
    px4_prefix_(px4_prefix),
    closed_loop_(node.declare_parameter<bool>("closed_loop", false)),
    calibration_confirmed_(node.declare_parameter<bool>(
        "calibration_confirmed", !configuration_.require_calibration_confirmation)),
    control_rate_hz_(node.declare_parameter<double>("control_rate_hz", 100.0)),
    state_timeout_s_(node.declare_parameter<double>("state_timeout_ms", 20.0) * 1.0e-3),
    vehicle_status_timeout_s_(
      node.declare_parameter<double>("vehicle_status_timeout_ms", 750.0) * 1.0e-3),
    land_state_timeout_s_(
      node.declare_parameter<double>("land_state_timeout_ms", 1500.0) * 1.0e-3),
    solver_budget_s_(node.declare_parameter<double>("solver_budget_ms", 8.0) * 1.0e-3),
    solver_timeout_limit_(node.declare_parameter<int>("solver_timeout_limit", 3)),
    allocator_status_timeout_s_(
      node.declare_parameter<double>("allocator_status_timeout_ms", 100.0) * 1.0e-3),
    follower_status_timeout_s_(
      node.declare_parameter<double>("follower_status_timeout_ms", 500.0) * 1.0e-3),
    fault_hold_s_(node.declare_parameter<double>("fault_hold_ms", 100.0) * 1.0e-3),
    stop_hold_s_(node.declare_parameter<double>("stop_hold_ms", 1000.0) * 1.0e-3),
    initial_gripper_open_fraction_(
      node.declare_parameter<double>("initial_gripper_open_fraction", 1.0)),
    target_frame_(node.declare_parameter<std::string>("target_frame", "px4_local_ned")),
    arm_joint_names_(node.declare_parameter<std::vector<std::string>>(
        "arm_joint_names", {"joint_1", "joint_2", "joint_3", "joint_4"})),
    gripper_joint_name_(node.declare_parameter<std::string>("gripper_joint_name", "joint_5")),
    arm_command_topic_(
      node.declare_parameter<std::string>("arm_command_topic", "/ace_leader/arm/command")),
    gripper_command_topic_(node.declare_parameter<std::string>(
        "gripper_command_topic", "/ace_leader/gripper/command")),
    sync_mode_topic_(
      node.declare_parameter<std::string>("sync_mode_topic", "/ace_leader/arm/sync_mode")),
    safety_(
      state_timeout_s_,
      solver_budget_s_,
      solver_timeout_limit_),
    controller_worker_(std::move(controller)),
    gripper_open_fraction_(initial_gripper_open_fraction_),
    locked_gripper_open_fraction_(initial_gripper_open_fraction_)
  {
    if (closed_loop_ && configuration_.require_calibration_confirmation &&
      !calibration_confirmed_)
    {
      throw std::invalid_argument(
              "Closed-loop Flying Hand control requires calibration_confirmed=true");
    }
    if (configuration_.mode_name.empty() || configuration_.diagnostic_name.empty() ||
      configuration_.hardware_id.empty() || configuration_.topic_prefix.empty() ||
      !end_effector_pose_flu_ || !std::isfinite(control_rate_hz_) || control_rate_hz_ <= 0.0 ||
      !std::isfinite(vehicle_status_timeout_s_) || vehicle_status_timeout_s_ <= 0.0 ||
      !std::isfinite(land_state_timeout_s_) || land_state_timeout_s_ <= 0.0 ||
      solver_timeout_limit_ <= 0 ||
      !std::isfinite(allocator_status_timeout_s_) || allocator_status_timeout_s_ <= 0.0 ||
      !std::isfinite(follower_status_timeout_s_) || follower_status_timeout_s_ <= 0.0 ||
      !std::isfinite(fault_hold_s_) || fault_hold_s_ < 0.0 ||
      !std::isfinite(stop_hold_s_) || stop_hold_s_ < 0.0 ||
      !std::isfinite(initial_gripper_open_fraction_) || initial_gripper_open_fraction_ < 0.0 ||
      initial_gripper_open_fraction_ > 1.0 || target_frame_.empty() ||
      arm_joint_names_.size() != static_cast<std::size_t>(kArmJointCount))
    {
      throw std::invalid_argument("Invalid Flying Hand runtime parameters");
    }

    wrench_setpoint_ = std::make_shared<WrenchSetpointType>(mode_);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    odometry_sub_ = node_.create_subscription<px4_msgs::msg::VehicleOdometry>(
      px4Topic<px4_msgs::msg::VehicleOdometry>(
        px4_prefix_, node_.declare_parameter<std::string>(
          "vehicle_odometry_topic", "/fmu/out/vehicle_odometry")),
      sensor_qos,
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr message) {
        onOdometry(*message);
      });
    sensor_sub_ = node_.create_subscription<px4_msgs::msg::SensorCombined>(
      px4Topic<px4_msgs::msg::SensorCombined>(
        px4_prefix_, node_.declare_parameter<std::string>(
          "sensor_combined_topic", "/fmu/out/sensor_combined")),
      sensor_qos,
      [this](const px4_msgs::msg::SensorCombined::SharedPtr message) {
        onSensorCombined(*message);
      });
    vehicle_status_sub_ = node_.create_subscription<px4_msgs::msg::VehicleStatus>(
      px4Topic<px4_msgs::msg::VehicleStatus>(
        px4_prefix_, node_.declare_parameter<std::string>(
          "vehicle_status_topic", "/fmu/out/vehicle_status")),
      sensor_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr message) {
        if (message->timestamp == 0 ||
        message->timestamp <= last_vehicle_status_source_timestamp_us_)
        {
          return;
        }
        last_vehicle_status_source_timestamp_us_ = message->timestamp;
        vehicle_status_received_ = true;
        vehicle_status_time_ = SteadyClock::now();
        armed_ = message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        rotary_wing_ =
        message->vehicle_type == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING;
        vehicle_failsafe_ = message->failsafe;
      });
    land_detected_sub_ = node_.create_subscription<px4_msgs::msg::VehicleLandDetected>(
      px4Topic<px4_msgs::msg::VehicleLandDetected>(
        px4_prefix_, node_.declare_parameter<std::string>(
          "vehicle_land_detected_topic", "/fmu/out/vehicle_land_detected")),
      sensor_qos,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr message) {
        if (message->timestamp == 0 || message->timestamp <= last_land_source_timestamp_us_) {
          return;
        }
        last_land_source_timestamp_us_ = message->timestamp;
        land_state_received_ = true;
        land_state_time_ = SteadyClock::now();
        airborne_ = !message->landed && !message->ground_contact;
      });

    const auto reliable_qos = rclcpp::QoS(1).reliable();
    follower_arm_state_sub_ = node_.create_subscription<sensor_msgs::msg::JointState>(
      node_.declare_parameter<std::string>(
        "follower_arm_state_topic", "/ace_follower/arm/state"),
      reliable_qos,
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        onFollowerArmState(*message);
      });
    follower_sync_status_sub_ = node_.create_subscription<std_msgs::msg::String>(
      node_.declare_parameter<std::string>(
        "follower_sync_status_topic", "/ace_follower/arm/sync_status"),
      reliable_qos,
      [this](const std_msgs::msg::String::SharedPtr message) {
        follower_sync_status_ = message->data;
        follower_sync_time_ = SteadyClock::now();
      });
    target_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
      node_.declare_parameter<std::string>(
        "ee_pose_setpoint_topic", configuration_.topic_prefix + "/ee_pose_setpoint"),
      reliable_qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        onTarget(*message);
      });

    arm_command_pub_ =
      node_.create_publisher<sensor_msgs::msg::JointState>(arm_command_topic_, reliable_qos);
    gripper_command_pub_ =
      node_.create_publisher<sensor_msgs::msg::JointState>(gripper_command_topic_, reliable_qos);
    sync_mode_pub_ = node_.create_publisher<std_msgs::msg::String>(sync_mode_topic_, reliable_qos);
    ee_pose_pub_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
      node_.declare_parameter<std::string>(
        "ee_pose_topic", configuration_.topic_prefix + "/ee_pose"), 10);
    nominal_wrench_pub_ = node_.create_publisher<geometry_msgs::msg::WrenchStamped>(
      node_.declare_parameter<std::string>(
        "nominal_wrench_topic", configuration_.topic_prefix + "/wrench_nominal"), 10);
    adaptive_wrench_pub_ = node_.create_publisher<geometry_msgs::msg::WrenchStamped>(
      node_.declare_parameter<std::string>(
        "adaptive_wrench_topic", configuration_.topic_prefix + "/wrench_adaptive"), 10);
    applied_wrench_pub_ = node_.create_publisher<geometry_msgs::msg::WrenchStamped>(
      node_.declare_parameter<std::string>(
        "applied_wrench_topic", configuration_.topic_prefix + "/wrench_applied"), 10);
    status_pub_ = node_.create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      node_.declare_parameter<std::string>(
        "status_topic", configuration_.topic_prefix + "/status"), 10);

    if (configuration_.monitor_control_allocator) {
      allocator_status_sub_ = node_.create_subscription<px4_msgs::msg::ControlAllocatorStatus>(
        px4Topic<px4_msgs::msg::ControlAllocatorStatus>(
          px4_prefix_, node_.declare_parameter<std::string>(
            "control_allocator_status_topic", "/fmu/out/control_allocator_status")),
        sensor_qos,
        [this](const px4_msgs::msg::ControlAllocatorStatus::SharedPtr message) {
          onControlAllocatorStatus(*message);
        });
    }

    const auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_rate_hz_));
    controller_timer_ = node_.create_wall_timer(control_period, [this]() {runController();});
    sync_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(100), [this]() {publishSyncState();});
    status_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(100), [this]() {publishStatus();});
  }

  ~Impl()
  {
    prepareShutdown();
  }

  void prepareShutdown() noexcept
  {
    if (shutting_down_.exchange(true)) {
      return;
    }
    controller_worker_.requestReset();
    try {
      publishSyncMode("stop", true);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        node_.get_logger(), "Failed to publish final Flying Hand stop: %s", exception.what());
    }
  }

  void setFallbackCallback(std::function<void(FaultReason)> callback)
  {
    fallback_callback_ = std::move(callback);
  }

  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter & reporter)
  {
    const FaultReason reason =
      safety_.faultLatched() ? safety_.faultReason() : safety_.readiness(safetySnapshot());
    if (reason != FaultReason::kNone) {
      /* EVENT
       * @description Flying Hand requires closed-loop enable, fresh vehicle/arm state, an
       * airborne multicopter, a ready follower, and an available controller.
       */
      reporter.armingCheckFailureExt(
        px4_ros2::events::ID("flying_hand_not_ready"), px4_ros2::events::Log::Error,
        "Flying Hand is not ready");
    }
  }

  void activate()
  {
    controller_worker_.requestRecovery();
    pending_output_.reset();
    safety_.reset();
    fault_time_.reset();
    fallback_notified_ = false;
    stop_until_.reset();
    last_accepted_sample_timestamp_us_.reset();
    inflight_sample_timing_.reset();
    last_submitted_sample_timestamp_us_ = 0;
    last_arm_command_rad_ = arm_position_rad_;
    locked_gripper_open_fraction_ = gripper_open_fraction_;
    allocator_command_started_ = false;
    allocator_saturation_count_ = 0;
    allocator_saturated_ = false;
    if (!target_ee_pose_ned_.has_value() && odometry_valid_ && arm_state_valid_) {
      target_ee_pose_ned_ = currentEePoseNed();
    }
    publishSyncMode("tracking");
  }

  void deactivate()
  {
    controller_worker_.rejectResult();
    last_accepted_sample_timestamp_us_.reset();
    inflight_sample_timing_.reset();
    pending_sample_timing_.reset();
    last_submitted_sample_timestamp_us_ = 0;
    controller_worker_.requestReset();
    pending_output_.reset();
    allocator_command_started_ = false;
    if (!hasArmCommandPublisherConflict()) {
      publishFinalArmHold();
      publishSyncMode("stop");
    }
    stop_until_ = SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(stop_hold_s_));
    safety_.reset();
    fault_time_.reset();
    fallback_notified_ = false;
    if (!explicit_target_received_) {
      target_ee_pose_ned_.reset();
    }
  }

  void updateSetpoint()
  {
    if (!closed_loop_ || shutting_down_.load()) {
      return;
    }

    if (pending_output_.has_value()) {
      const ControllerOutput output = *pending_output_;
      SafetySnapshot snapshot = safetySnapshot();
      snapshot.controller_output_ready = true;
      snapshot.controller_output_age_s = 0.0;
      FaultReason publication_reason = safety_.readiness(snapshot);
      if (publication_reason == FaultReason::kNone &&
        (!pending_sample_timing_.has_value() ||
        pending_sample_timing_->stale(SteadyClock::now(), state_timeout_s_)))
      {
        publication_reason = FaultReason::kOdometryStale;
      }
      if (publication_reason != FaultReason::kNone ||
        !pending_sample_timing_.has_value() ||
        output.sample_timestamp_us != pending_sample_timing_->timestamp_us)
      {
        controller_worker_.rejectResult();
        controller_worker_.requestReset();
        pending_output_.reset();
        last_accepted_sample_timestamp_us_.reset();
        inflight_sample_timing_.reset();
        pending_sample_timing_.reset();
        last_submitted_sample_timestamp_us_ = 0;
        triggerFault(
          publication_reason != FaultReason::kNone ?
          publication_reason : FaultReason::kControllerOutputMissing);
        return;
      }
      if (!controller_worker_.acceptResult()) {
        pending_output_.reset();
        inflight_sample_timing_.reset();
        pending_sample_timing_.reset();
        triggerFault(FaultReason::kControllerOutputMissing);
        return;
      }
      last_accepted_sample_timestamp_us_ = output.sample_timestamp_us;
      inflight_sample_timing_.reset();
      pending_output_.reset();

      if (!wrench_setpoint_->update(
          output.normalized_thrust_frd, output.normalized_torque_frd,
          output.sample_timestamp_us))
      {
        triggerFault(FaultReason::kSetpointRejected);
        return;
      }
      allocator_command_started_ = true;

      try {
        publishControllerDiagnostics(output, pending_output_stamp_);
        publishArmCommand(output.arm_position_command_rad, pending_output_stamp_);
        publishGripperCommand(pending_output_stamp_);
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          node_.get_logger(), "Failed to publish Flying Hand whole-body command: %s",
          exception.what());
        triggerFault(FaultReason::kSetpointRejected);
        return;
      }

      last_feasible_output_ = output;
      last_output_sample_timing_ = pending_sample_timing_;
      pending_sample_timing_.reset();
      last_output_time_ = SteadyClock::now();
      last_output_stamp_ = pending_output_stamp_;
      submitControllerSample();
      return;
    }

    if (!last_feasible_output_.has_value()) {
      return;
    }

    const SteadyTime now = SteadyClock::now();
    if (!safety_.faultLatched()) {
      FaultReason publication_reason = safety_.readiness(safetySnapshot());
      if (publication_reason == FaultReason::kNone &&
        (!last_output_sample_timing_.has_value() ||
        last_output_sample_timing_->stale(now, state_timeout_s_)))
      {
        publication_reason = FaultReason::kControllerOutputStale;
      }
      if (publication_reason != FaultReason::kNone) {
        triggerFault(publication_reason);
      }
    }

    if (safety_.faultLatched()) {
      if (safety_.faultReason() == FaultReason::kExternalCommandPublisher) {
        return;
      }
      if (!fault_time_.has_value() ||
        std::chrono::duration<double>(now - *fault_time_).count() > fault_hold_s_)
      {
        return;
      }
    }

    if (!wrench_setpoint_->update(
        last_feasible_output_->normalized_thrust_frd,
        last_feasible_output_->normalized_torque_frd,
        last_feasible_output_->sample_timestamp_us))
    {
      triggerFault(FaultReason::kSetpointRejected);
      return;
    }

    try {
      const rclcpp::Time stamp = node_.now();
      publishArmCommand(last_feasible_output_->arm_position_command_rad, stamp);
      publishGripperCommand(stamp);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        node_.get_logger(), "Failed to republish Flying Hand arm hold: %s", exception.what());
      triggerFault(FaultReason::kSetpointRejected);
    }
  }

private:
  void onOdometry(const px4_msgs::msg::VehicleOdometry & message)
  {
    if (message.timestamp == 0 || message.timestamp_sample == 0 ||
      message.timestamp < message.timestamp_sample ||
      message.timestamp <= last_odometry_source_timestamp_us_ ||
      message.timestamp_sample <= sample_timestamp_us_)
    {
      return;
    }
    const Eigen::Vector3d position(
      message.position[0], message.position[1], message.position[2]);
    Eigen::Quaterniond attitude(message.q[0], message.q[1], message.q[2], message.q[3]);
    Eigen::Vector3d velocity(message.velocity[0], message.velocity[1], message.velocity[2]);
    const Eigen::Vector3d angular_velocity(
      message.angular_velocity[0], message.angular_velocity[1], message.angular_velocity[2]);

    odometry_valid_ =
      message.pose_frame == px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED &&
      position.allFinite() && finiteQuaternion(attitude) && velocity.allFinite() &&
      angular_velocity.allFinite();
    if (odometry_valid_) {
      attitude.normalize();
      const bool velocity_is_ned =
        message.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
      if (message.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
        velocity = attitude * velocity;
      } else if (!velocity_is_ned) {
        odometry_valid_ = false;
      }
    }
    if (!odometry_valid_) {
      return;
    }

    sample_timestamp_us_ = message.timestamp_sample;
    last_odometry_source_timestamp_us_ = message.timestamp;
    odometry_source_age_at_receipt_s_ =
      static_cast<double>(message.timestamp - message.timestamp_sample) * 1.0e-6;
    position_ned_m_ = position;
    attitude_ned_frd_ = attitude;
    linear_velocity_ned_m_s_ = velocity;
    angular_velocity_frd_rad_s_ = angular_velocity;
    odometry_time_ = SteadyClock::now();
  }

  void onSensorCombined(const px4_msgs::msg::SensorCombined & message)
  {
    if (message.timestamp == 0 || message.timestamp <= last_imu_source_timestamp_us_) {
      return;
    }
    const Eigen::Vector3d gyro(
      message.gyro_rad[0], message.gyro_rad[1], message.gyro_rad[2]);
    imu_valid_ = gyro.allFinite();
    if (!imu_valid_) {
      return;
    }
    gyro_frd_rad_s_ = gyro;
    last_imu_source_timestamp_us_ = message.timestamp;
    imu_time_ = SteadyClock::now();
  }

  void onControlAllocatorStatus(const px4_msgs::msg::ControlAllocatorStatus & message)
  {
    if (message.timestamp == 0 || message.timestamp <= last_allocator_source_timestamp_us_) {
      return;
    }
    last_allocator_source_timestamp_us_ = message.timestamp;
    allocator_status_received_ = true;
    allocator_status_time_ = SteadyClock::now();
    allocator_setpoint_achieved_ =
      message.thrust_setpoint_achieved && message.torque_setpoint_achieved;
    const bool saturated = std::any_of(
      message.actuator_saturation.begin(), message.actuator_saturation.end(),
      [](std::int8_t value) {return value != 0;});
    allocator_saturation_count_ = saturated ? allocator_saturation_count_ + 1 : 0;
    allocator_saturated_ = allocator_saturation_count_ >= kAllocatorSaturationLimit;
  }

  void onFollowerArmState(const sensor_msgs::msg::JointState & message)
  {
    const std::int64_t source_timestamp_ns = rclcpp::Time(message.header.stamp).nanoseconds();
    if (source_timestamp_ns <= 0 || source_timestamp_ns <= last_arm_source_timestamp_ns_) {
      arm_state_valid_ = false;
      return;
    }
    JointVector positions = JointVector::Zero();
    JointVector velocities = JointVector::Zero();

    for (std::size_t joint = 0; joint < arm_joint_names_.size(); ++joint) {
      std::size_t source_index = joint;
      if (!message.name.empty()) {
        const auto iterator =
          std::find(message.name.begin(), message.name.end(), arm_joint_names_[joint]);
        if (iterator == message.name.end()) {
          arm_state_valid_ = false;
          return;
        }
        source_index = static_cast<std::size_t>(std::distance(message.name.begin(), iterator));
      }
      if (source_index >= message.position.size()) {
        arm_state_valid_ = false;
        return;
      }
      if (source_index >= message.velocity.size()) {
        arm_state_valid_ = false;
        return;
      }
      positions[static_cast<Eigen::Index>(joint)] = message.position[source_index];
      velocities[static_cast<Eigen::Index>(joint)] = message.velocity[source_index];
    }

    const bool has_positional_gripper =
      message.position.size() > static_cast<std::size_t>(kArmJointCount) &&
      std::isfinite(message.position[kArmJointCount]);
    if (!message.name.empty()) {
      const auto gripper =
        std::find(message.name.begin(), message.name.end(), gripper_joint_name_);
      if (gripper != message.name.end()) {
        const auto index = static_cast<std::size_t>(std::distance(message.name.begin(), gripper));
        if (index < message.position.size() && std::isfinite(message.position[index])) {
          gripper_open_fraction_ = std::clamp(message.position[index], 0.0, 1.0);
        }
      }
    } else if (has_positional_gripper) {
      gripper_open_fraction_ = std::clamp(message.position[kArmJointCount], 0.0, 1.0);
    }

    arm_state_valid_ = positions.allFinite() && velocities.allFinite();
    if (!arm_state_valid_) {
      return;
    }
    arm_position_rad_ = positions;
    arm_velocity_rad_s_ = velocities;
    last_arm_source_timestamp_ns_ = source_timestamp_ns;
    arm_state_time_ = SteadyClock::now();
  }

  void onTarget(const geometry_msgs::msg::PoseStamped & message)
  {
    if (message.header.frame_id != target_frame_) {
      RCLCPP_WARN(
        node_.get_logger(), "Ignoring Flying Hand target in frame '%s'; expected '%s'",
        message.header.frame_id.c_str(), target_frame_.c_str());
      return;
    }

    const Eigen::Vector3d position(
      message.pose.position.x, message.pose.position.y, message.pose.position.z);
    Eigen::Quaterniond orientation(
      message.pose.orientation.w, message.pose.orientation.x,
      message.pose.orientation.y, message.pose.orientation.z);
    if (!position.allFinite() || !finiteQuaternion(orientation)) {
      RCLCPP_WARN(node_.get_logger(), "Ignoring non-finite Flying Hand EE pose target");
      return;
    }

    orientation.normalize();
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = position;
    target.linear() = orientation.toRotationMatrix();
    target_ee_pose_ned_ = target;
    explicit_target_received_ = true;
    if (!mode_.isActive()) {
      last_feasible_output_.reset();
      last_output_time_ = SteadyTime{};
    }
  }

  Eigen::Isometry3d currentEePoseNed() const
  {
    Eigen::Isometry3d base_ned_frd = Eigen::Isometry3d::Identity();
    base_ned_frd.translation() = position_ned_m_;
    base_ned_frd.linear() = attitude_ned_frd_.toRotationMatrix();

    Eigen::Isometry3d frd_flu = Eigen::Isometry3d::Identity();
    frd_flu.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
    return base_ned_frd * frd_flu * end_effector_pose_flu_(arm_position_rad_);
  }

  SafetySnapshot safetySnapshot() const
  {
    const SteadyTime now = SteadyClock::now();
    const bool follower_status_recent =
      ageSeconds(follower_sync_time_, now) <= follower_status_timeout_s_;
    const bool follower_ready = follower_status_recent &&
      (follower_sync_status_ == "ready" || follower_sync_status_ == "tracking");

    SafetySnapshot snapshot;
    snapshot.closed_loop = closed_loop_;
    snapshot.calibration_confirmed =
      !configuration_.require_calibration_confirmation || calibration_confirmed_;
    snapshot.controller_available = controller_worker_.valid();
    snapshot.controller_output_ready = last_feasible_output_.has_value();
    snapshot.vehicle_status_received = vehicle_status_received_ &&
      ageSeconds(vehicle_status_time_, now) <= vehicle_status_timeout_s_;
    snapshot.armed = armed_;
    snapshot.rotary_wing = rotary_wing_;
    snapshot.vehicle_failsafe = vehicle_failsafe_;
    snapshot.land_state_received = land_state_received_ &&
      ageSeconds(land_state_time_, now) <= land_state_timeout_s_;
    snapshot.airborne = airborne_;
    snapshot.odometry_valid = odometry_valid_;
    snapshot.imu_valid = imu_valid_;
    snapshot.arm_state_valid = arm_state_valid_;
    snapshot.follower_ready = follower_ready;
    snapshot.allocator_status_required = configuration_.monitor_control_allocator;
    snapshot.allocator_status_received = allocator_status_received_ &&
      ageSeconds(allocator_status_time_, now) <= allocator_status_timeout_s_;
    snapshot.allocator_setpoint_achieved =
      !allocator_command_started_ || allocator_setpoint_achieved_;
    snapshot.allocator_saturated = allocator_command_started_ && allocator_saturated_;
    snapshot.external_command_publisher = hasConflictingCommandPublisher();
    snapshot.odometry_age_s =
      odometry_source_age_at_receipt_s_ + ageSeconds(odometry_time_, now);
    snapshot.imu_age_s = ageSeconds(imu_time_, now);
    snapshot.arm_state_age_s = ageSeconds(arm_state_time_, now);
    snapshot.allocator_status_age_s = ageSeconds(allocator_status_time_, now);
    snapshot.controller_output_age_s = ageSeconds(last_output_time_, now);
    return snapshot;
  }

  bool ownedTopicHasConflict(const std::string & topic) const
  {
    const auto endpoints = node_.get_publishers_info_by_topic(topic);
    std::size_t own_publishers = 0;
    for (const auto & endpoint : endpoints) {
      if (endpoint.node_name() == node_.get_name() &&
        endpoint.node_namespace() == node_.get_namespace())
      {
        ++own_publishers;
      } else {
        return true;
      }
    }
    return endpoints.size() != 1 || own_publishers != 1;
  }

  bool hasArmCommandPublisherConflict() const
  {
    const std::array<std::string, 3> command_topics{
      arm_command_pub_->get_topic_name(), gripper_command_pub_->get_topic_name(),
      sync_mode_pub_->get_topic_name()};
    for (const std::string & topic : command_topics) {
      if (ownedTopicHasConflict(topic)) {
        return true;
      }
    }
    return false;
  }

  bool hasConflictingCommandPublisher() const
  {
    if (hasArmCommandPublisherConflict() ||
      ownedTopicHasConflict(wrench_setpoint_->thrustTopic()) ||
      ownedTopicHasConflict(wrench_setpoint_->torqueTopic()))
    {
      return true;
    }
    if (node_.get_publishers_info_by_topic(target_sub_->get_topic_name()).size() > 1) {
      return true;
    }
    return false;
  }

  void runController()
  {
    if (shutting_down_.load()) {
      return;
    }
    publishCurrentEePose();

    if (pending_output_.has_value()) {
      return;
    }

    if (const auto worker_result = controller_worker_.result()) {
      last_solver_elapsed_s_ = worker_result->elapsed_s;
      const bool output_valid =
        worker_result->output.feasible && worker_result->output.normalizedCommandValid();
      const bool accepted_by_watchdog =
        safety_.recordSolverResult(last_solver_elapsed_s_, output_valid);
      if (!accepted_by_watchdog) {
        const bool recovered = mode_.isActive() && closed_loop_ ?
          controller_worker_.recoverResult() : controller_worker_.rejectResult();
        inflight_sample_timing_.reset();
        if (!recovered) {
          triggerFault(FaultReason::kControllerOutputMissing);
          return;
        }
        if (safety_.faultLatched() && mode_.isActive() && closed_loop_) {
          triggerFault(safety_.faultReason());
        } else if (!mode_.isActive()) {
          last_shadow_fault_ = safety_.faultLatched() ?
            safety_.faultReason() :
            (output_valid ? FaultReason::kSolverTimeout : FaultReason::kSolverInvalid);
          last_shadow_fault_time_ = SteadyClock::now();
          safety_.reset();
        }
        if (!safety_.faultLatched()) {
          if (mode_.isActive() && closed_loop_) {
            last_accepted_sample_timestamp_us_.reset();
            last_submitted_sample_timestamp_us_ = 0;
          }
          submitControllerSample();
        }
        return;
      }

      if (mode_.isActive() && closed_loop_) {
        pending_output_ = worker_result->output;
        pending_output_stamp_ = node_.now();
        pending_sample_timing_ = inflight_sample_timing_;
      } else {
        if (!controller_worker_.acceptResult()) {
          controller_worker_.requestReset();
          last_shadow_fault_ = FaultReason::kControllerOutputMissing;
          last_shadow_fault_time_ = SteadyClock::now();
          last_accepted_sample_timestamp_us_.reset();
          inflight_sample_timing_.reset();
          last_submitted_sample_timestamp_us_ = 0;
          return;
        }
        last_accepted_sample_timestamp_us_ = worker_result->output.sample_timestamp_us;
        last_output_sample_timing_ = inflight_sample_timing_;
        inflight_sample_timing_.reset();
        last_shadow_fault_ = FaultReason::kNone;
        last_shadow_fault_time_ = SteadyTime{};
        last_feasible_output_ = worker_result->output;
        last_output_time_ = SteadyClock::now();
        last_output_stamp_ = node_.now();
        publishControllerDiagnostics(worker_result->output, last_output_stamp_);
        submitControllerSample();
      }
      return;
    }

    if (controller_worker_.busy()) {
      const double elapsed_s = controller_worker_.busyElapsedS();
      last_solver_elapsed_s_ = elapsed_s;
      if (elapsed_s > solver_budget_s_ * static_cast<double>(solver_timeout_limit_) &&
        !safety_.faultLatched())
      {
        safety_.latch(FaultReason::kSolverTimeout);
        controller_worker_.requestReset();
        last_accepted_sample_timestamp_us_.reset();
        inflight_sample_timing_.reset();
        last_submitted_sample_timestamp_us_ = 0;
        if (mode_.isActive() && closed_loop_) {
          triggerFault(safety_.faultReason());
        } else {
          last_shadow_fault_ = FaultReason::kSolverTimeout;
          last_shadow_fault_time_ = SteadyClock::now();
        }
      }
      return;
    }

    submitControllerSample();
  }

  void submitControllerSample()
  {
    if (pending_output_.has_value() || controller_worker_.busy() ||
      controller_worker_.result().has_value())
    {
      return;
    }
    if (sample_timestamp_us_ == 0 || sample_timestamp_us_ <= last_submitted_sample_timestamp_us_ ||
      (last_accepted_sample_timestamp_us_.has_value() &&
      sample_timestamp_us_ <= *last_accepted_sample_timestamp_us_))
    {
      return;
    }

    SafetySnapshot snapshot = safetySnapshot();
    snapshot.closed_loop = true;  // Shadow mode still evaluates the controller when flight-ready.
    snapshot.calibration_confirmed = true;
    snapshot.allocator_status_required = false;
    snapshot.controller_output_ready = true;  // The current solve establishes output readiness.
    snapshot.controller_output_age_s = 0.0;
    if (!mode_.isActive() && safety_.faultLatched()) {
      safety_.reset();
    }
    const FaultReason state_reason = safety_.readiness(snapshot);
    if (state_reason != FaultReason::kNone) {
      if (mode_.isActive() && closed_loop_) {
        triggerFault(state_reason);
      }
      return;
    }

    double dt_s = 1.0 / control_rate_hz_;
    if (last_accepted_sample_timestamp_us_.has_value()) {
      dt_s = std::clamp(
        static_cast<double>(sample_timestamp_us_ - *last_accepted_sample_timestamp_us_) * 1.0e-6,
        1.0e-4, 0.1);
    }

    ControllerInput input;
    input.sample_timestamp_us = sample_timestamp_us_;
    input.dt_s = dt_s;
    input.position_ned_m = position_ned_m_;
    input.attitude_ned_frd = attitude_ned_frd_;
    input.linear_velocity_ned_m_s = linear_velocity_ned_m_s_;
    input.angular_velocity_frd_rad_s = angular_velocity_frd_rad_s_;
    input.gyro_frd_rad_s = gyro_frd_rad_s_;
    input.arm_position_rad = arm_position_rad_;
    input.arm_velocity_rad_s = arm_velocity_rad_s_;
    input.current_ee_pose_ned = currentEePoseNed();
    input.target_ee_pose_ned = target_ee_pose_ned_.value_or(input.current_ee_pose_ned);

    if (controller_worker_.submit(input)) {
      ControllerSampleTiming timing;
      timing.timestamp_us = sample_timestamp_us_;
      timing.oldest_receipt_time = std::min({odometry_time_, imu_time_, arm_state_time_});
      timing.odometry_receipt_time = odometry_time_;
      timing.odometry_source_age_at_receipt_s = odometry_source_age_at_receipt_s_;
      inflight_sample_timing_ = timing;
      last_submitted_sample_timestamp_us_ = sample_timestamp_us_;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 1000,
        "Flying Hand controller worker is not ready for a new sample");
    }
  }

  void triggerFault(FaultReason reason)
  {
    if (pending_output_.has_value()) {
      controller_worker_.rejectResult();
      pending_output_.reset();
    }
    controller_worker_.requestReset();
    last_accepted_sample_timestamp_us_.reset();
    inflight_sample_timing_.reset();
    pending_sample_timing_.reset();
    last_submitted_sample_timestamp_us_ = 0;
    safety_.latch(reason);
    if (!fault_time_.has_value()) {
      fault_time_ = SteadyClock::now();
    }
    if (fallback_notified_) {
      return;
    }

    fallback_notified_ = true;
    if (!hasArmCommandPublisherConflict()) {
      try {
        publishFinalArmHold();
        publishSyncMode("stop");
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          node_.get_logger(), "Failed to publish Flying Hand fault hold: %s", exception.what());
      }
    }
    stop_until_ = SteadyClock::now() +
      std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(stop_hold_s_));
    RCLCPP_ERROR(
      node_.get_logger(), "Flying Hand fault latched: %s", faultReasonName(safety_.faultReason()));
    if (fallback_callback_) {
      fallback_callback_(safety_.faultReason());
    }
  }

  void publishControllerDiagnostics(
    const ControllerOutput & output, const rclcpp::Time & stamp)
  {
    nominal_wrench_pub_->publish(makeWrenchMessage(stamp, output.nominal_wrench_flu));
    adaptive_wrench_pub_->publish(makeWrenchMessage(stamp, output.adaptive_wrench_flu));
    applied_wrench_pub_->publish(makeWrenchMessage(stamp, output.applied_wrench_flu));
  }

  void publishArmCommand(const JointVector & command, const rclcpp::Time & stamp)
  {
    if (!closed_loop_) {
      return;
    }
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp;
    message.name = arm_joint_names_;
    message.position.assign(command.data(), command.data() + command.size());
    message.velocity.assign(kArmJointCount, 0.0);
    message.effort.assign(kArmJointCount, 0.0);
    arm_command_pub_->publish(message);
    last_arm_command_rad_ = command;
  }

  void publishGripperCommand(const rclcpp::Time & stamp)
  {
    if (!closed_loop_) {
      return;
    }
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp;
    message.name = {gripper_joint_name_};
    message.position = {locked_gripper_open_fraction_};
    message.velocity = {0.0};
    message.effort = {0.0};
    gripper_command_pub_->publish(message);
  }

  void publishFinalArmHold()
  {
    const rclcpp::Time stamp = node_.now();
    const bool current_arm_state_is_fresh = arm_state_valid_ &&
      ageSeconds(arm_state_time_, SteadyClock::now()) <= state_timeout_s_;
    publishArmCommand(
      current_arm_state_is_fresh ? arm_position_rad_ :
      last_arm_command_rad_.value_or(arm_position_rad_), stamp);
    publishGripperCommand(stamp);
  }

  void publishSyncMode(const std::string & state, bool allow_during_shutdown = false)
  {
    std::lock_guard<std::mutex> lock(sync_publish_mutex_);
    if (shutting_down_.load() && !allow_during_shutdown) {
      return;
    }
    std_msgs::msg::String message;
    message.data = state;
    sync_mode_pub_->publish(message);
  }

  void publishSyncState()
  {
    if (shutting_down_.load()) {
      return;
    }
    const SteadyTime now = SteadyClock::now();
    if (hasArmCommandPublisherConflict()) {
      return;
    }
    if (stop_until_.has_value() && now < *stop_until_) {
      publishSyncMode("stop");
      return;
    }
    stop_until_.reset();
    if (mode_.isActive() && closed_loop_ && !safety_.faultLatched()) {
      publishSyncMode("tracking");
    } else {
      publishSyncMode("sync_request");
    }
  }

  void publishCurrentEePose()
  {
    if (!odometry_valid_ || !arm_state_valid_) {
      return;
    }
    const Eigen::Isometry3d pose = currentEePoseNed();
    Eigen::Quaterniond orientation(pose.linear());
    orientation.normalize();

    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = node_.now();
    message.header.frame_id = target_frame_;
    message.pose.position.x = pose.translation().x();
    message.pose.position.y = pose.translation().y();
    message.pose.position.z = pose.translation().z();
    message.pose.orientation.w = orientation.w();
    message.pose.orientation.x = orientation.x();
    message.pose.orientation.y = orientation.y();
    message.pose.orientation.z = orientation.z();
    ee_pose_pub_->publish(message);
  }

  void publishStatus()
  {
    const SafetySnapshot snapshot = safetySnapshot();
    const FaultReason readiness = safety_.readiness(snapshot);
    const FaultReason reason = safety_.faultLatched() ? safety_.faultReason() : readiness;

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = node_.now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = configuration_.diagnostic_name;
    status.hardware_id = configuration_.hardware_id;
    status.level = safety_.faultLatched() ?
      diagnostic_msgs::msg::DiagnosticStatus::ERROR :
      (reason == FaultReason::kNone ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN);
    status.message = faultReasonName(reason);

    const auto add_value = [&status](const std::string & key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue entry;
        entry.key = key;
        entry.value = value;
        status.values.push_back(std::move(entry));
      };
    add_value("active", mode_.isActive() ? "true" : "false");
    add_value("closed_loop", closed_loop_ ? "true" : "false");
    add_value("calibration_confirmed", calibration_confirmed_ ? "true" : "false");
    add_value("controller_available", controller_worker_.valid() ? "true" : "false");
    add_value(
      "controller_output_ready", snapshot.controller_output_ready ? "true" : "false");
    add_value("follower_ready", snapshot.follower_ready ? "true" : "false");
    add_value(
      "allocator_status_received", snapshot.allocator_status_received ? "true" : "false");
    add_value(
      "allocator_setpoint_achieved",
      snapshot.allocator_setpoint_achieved ? "true" : "false");
    add_value("allocator_saturated", snapshot.allocator_saturated ? "true" : "false");
    add_value(
      "external_command_publisher",
      snapshot.external_command_publisher ? "true" : "false");
    add_value("target_source", explicit_target_received_ ? "topic" : "latched_current_pose");
    add_value("odometry_age_ms", std::to_string(snapshot.odometry_age_s * 1.0e3));
    add_value("imu_age_ms", std::to_string(snapshot.imu_age_s * 1.0e3));
    add_value("arm_state_age_ms", std::to_string(snapshot.arm_state_age_s * 1.0e3));
    add_value(
      "allocator_status_age_ms", std::to_string(snapshot.allocator_status_age_s * 1.0e3));
    add_value(
      "controller_output_age_ms", std::to_string(snapshot.controller_output_age_s * 1.0e3));
    add_value("solver_elapsed_ms", std::to_string(last_solver_elapsed_s_ * 1.0e3));
    if (last_feasible_output_.has_value()) {
      add_value("tracking_cost", std::to_string(last_feasible_output_->tracking_cost));
      add_value(
        "allocation_condition_number",
        std::to_string(last_feasible_output_->allocation_condition_number));
      add_value(
        "rotor_saturated", last_feasible_output_->rotor_saturated ? "true" : "false");
    }
    add_value("solver_consecutive_timeouts", std::to_string(safety_.consecutiveTimeouts()));
    add_value("last_shadow_fault", faultReasonName(last_shadow_fault_));
    add_value(
      "last_shadow_fault_age_ms",
      std::to_string(ageSeconds(last_shadow_fault_time_, SteadyClock::now()) * 1.0e3));
    array.status.push_back(std::move(status));
    status_pub_->publish(array);
  }

  FlyingHandMode & mode_;
  rclcpp::Node & node_;
  const FlyingHandModeConfiguration configuration_;
  const EndEffectorPoseCallback end_effector_pose_flu_;
  const std::string px4_prefix_;
  const bool closed_loop_;
  const bool calibration_confirmed_;
  const double control_rate_hz_;
  const double state_timeout_s_;
  const double vehicle_status_timeout_s_;
  const double land_state_timeout_s_;
  const double solver_budget_s_;
  const int solver_timeout_limit_;
  const double allocator_status_timeout_s_;
  const double follower_status_timeout_s_;
  const double fault_hold_s_;
  const double stop_hold_s_;
  const double initial_gripper_open_fraction_;
  const std::string target_frame_;
  const std::vector<std::string> arm_joint_names_;
  const std::string gripper_joint_name_;
  const std::string arm_command_topic_;
  const std::string gripper_command_topic_;
  const std::string sync_mode_topic_;

  FlyingHandSafety safety_;
  ControllerWorker controller_worker_;
  std::function<void(FaultReason)> fallback_callback_;
  std::shared_ptr<WrenchSetpointType> wrench_setpoint_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::Subscription<px4_msgs::msg::ControlAllocatorStatus>::SharedPtr allocator_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_arm_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr follower_sync_status_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sync_mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr nominal_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr adaptive_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr applied_wrench_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr controller_timer_;
  rclcpp::TimerBase::SharedPtr sync_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  std::uint64_t sample_timestamp_us_{0};
  std::uint64_t last_odometry_source_timestamp_us_{0};
  std::uint64_t last_imu_source_timestamp_us_{0};
  std::uint64_t last_vehicle_status_source_timestamp_us_{0};
  std::uint64_t last_land_source_timestamp_us_{0};
  std::uint64_t last_allocator_source_timestamp_us_{0};
  std::uint64_t last_submitted_sample_timestamp_us_{0};
  std::int64_t last_arm_source_timestamp_ns_{0};
  Eigen::Vector3d position_ned_m_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude_ned_frd_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d linear_velocity_ned_m_s_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_frd_rad_s_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_frd_rad_s_{Eigen::Vector3d::Zero()};
  JointVector arm_position_rad_{JointVector::Zero()};
  JointVector arm_velocity_rad_s_{JointVector::Zero()};
  std::optional<JointVector> last_arm_command_rad_;
  std::optional<Eigen::Isometry3d> target_ee_pose_ned_;
  std::optional<ControllerOutput> last_feasible_output_;
  std::optional<ControllerOutput> pending_output_;
  std::optional<ControllerSampleTiming> inflight_sample_timing_;
  std::optional<ControllerSampleTiming> pending_sample_timing_;
  std::optional<ControllerSampleTiming> last_output_sample_timing_;
  rclcpp::Time pending_output_stamp_{0, 0, RCL_ROS_TIME};

  bool vehicle_status_received_{false};
  bool armed_{false};
  bool rotary_wing_{false};
  bool vehicle_failsafe_{false};
  bool land_state_received_{false};
  bool airborne_{false};
  bool odometry_valid_{false};
  bool imu_valid_{false};
  bool arm_state_valid_{false};
  bool allocator_status_received_{false};
  bool allocator_setpoint_achieved_{true};
  bool allocator_saturated_{false};
  bool allocator_command_started_{false};
  int allocator_saturation_count_{0};
  bool explicit_target_received_{false};
  bool fallback_notified_{false};
  std::atomic<bool> shutting_down_{false};
  std::mutex sync_publish_mutex_;
  FaultReason last_shadow_fault_{FaultReason::kNone};
  std::string follower_sync_status_{"idle"};
  double gripper_open_fraction_;
  double locked_gripper_open_fraction_;
  double last_solver_elapsed_s_{0.0};
  double odometry_source_age_at_receipt_s_{0.0};
  rclcpp::Time last_output_stamp_{0, 0, RCL_ROS_TIME};

  SteadyTime odometry_time_{};
  SteadyTime imu_time_{};
  SteadyTime arm_state_time_{};
  SteadyTime follower_sync_time_{};
  SteadyTime vehicle_status_time_{};
  SteadyTime land_state_time_{};
  SteadyTime allocator_status_time_{};
  std::optional<std::uint64_t> last_accepted_sample_timestamp_us_;
  SteadyTime last_output_time_{};
  SteadyTime last_shadow_fault_time_{};
  std::optional<SteadyTime> fault_time_;
  std::optional<SteadyTime> stop_until_;
};

FlyingHandMode::FlyingHandMode(
  rclcpp::Node & node, FlyingHandModeConfiguration configuration,
  EndEffectorPoseCallback end_effector_pose_flu,
  const std::string & px4_topic_namespace_prefix, ControllerCallbacks controller)
: ModeBase(
    node, Settings{configuration.mode_name}.preventArming(true),
    px4_topic_namespace_prefix),
  impl_(std::make_unique<Impl>(
      *this, node, std::move(configuration), std::move(end_effector_pose_flu),
      px4_topic_namespace_prefix, std::move(controller)))
{
  modeRequirements().angular_velocity = true;
  modeRequirements().attitude = true;
  modeRequirements().local_position = true;
  modeRequirements().prevent_arming = true;
}

FlyingHandMode::~FlyingHandMode() = default;

void FlyingHandMode::setFallbackCallback(std::function<void(FaultReason)> callback)
{
  impl_->setFallbackCallback(std::move(callback));
}

void FlyingHandMode::checkArmingAndRunConditions(
  px4_ros2::HealthAndArmingCheckReporter & reporter)
{
  impl_->checkArmingAndRunConditions(reporter);
}

void FlyingHandMode::onActivate()
{
  impl_->activate();
}

void FlyingHandMode::onDeactivate()
{
  impl_->deactivate();
}

void FlyingHandMode::updateSetpoint(float dt_s)
{
  (void)dt_s;
  impl_->updateSetpoint();
}

void FlyingHandMode::prepareShutdown() noexcept
{
  impl_->prepareShutdown();
}

class FlyingHandModeExecutor::Impl
{
public:
  Impl(FlyingHandModeExecutor & executor, const std::string & px4_prefix)
  : executor_(executor)
  {
    command_publisher_ = executor_.node().create_publisher<px4_msgs::msg::VehicleCommand>(
      px4Topic<px4_msgs::msg::VehicleCommand>(
        px4_prefix, "/fmu/in/vehicle_command_mode_executor"), 1);
    vehicle_status_sub_ =
      executor_.node().create_subscription<px4_msgs::msg::VehicleStatus>(
      px4Topic<px4_msgs::msg::VehicleStatus>(px4_prefix, "/fmu/out/vehicle_status"),
      rclcpp::SensorDataQoS().keep_last(1),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr message) {
        if (requested_ &&
        message->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL)
        {
          requested_ = false;
          failure_reported_ = false;
          RCLCPP_INFO(
            executor_.node().get_logger(), "Flying Hand Position fallback confirmed");
        }
      });
    retry_timer_ = executor_.node().create_wall_timer(
      std::chrono::milliseconds(100), [this]() {publishFallbackRequest();});
  }

  void activate()
  {
    requested_ = false;
    attempts_ = 0;
    failure_reported_ = false;
    last_request_time_ = SteadyTime{};
  }

  void deactivate()
  {
    requested_ = false;
    attempts_ = 0;
    failure_reported_ = false;
    last_request_time_ = SteadyTime{};
  }

  void request(FaultReason reason)
  {
    if (reason == FaultReason::kVehicleFailsafe) {
      RCLCPP_ERROR(
        executor_.node().get_logger(),
        "PX4 failsafe is already active; Flying Hand will not override its fallback mode");
      return;
    }
    if (requested_ || !executor_.isInCharge()) {
      return;
    }

    requested_ = true;
    attempts_ = 0;
    RCLCPP_ERROR(
      executor_.node().get_logger(),
      "Requesting non-blocking Position fallback after Flying Hand fault: %s",
      faultReasonName(reason));
    publishFallbackRequest();
  }

private:
  void publishFallbackRequest()
  {
    constexpr int kMaximumAttempts = 3;
    if (!requested_ || !executor_.isInCharge()) {
      return;
    }
    const SteadyTime now = SteadyClock::now();
    if (last_request_time_ != SteadyTime{} &&
      std::chrono::duration<double>(now - last_request_time_).count() < 1.0)
    {
      return;
    }
    if (attempts_ >= kMaximumAttempts && !failure_reported_) {
      failure_reported_ = true;
      RCLCPP_ERROR(
        executor_.node().get_logger(),
        "Flying Hand Position fallback was not confirmed after %d requests; retrying at 1 Hz",
        kMaximumAttempts);
    }

    px4_msgs::msg::VehicleCommand command{};
    command.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE;
    command.param1 = static_cast<float>(px4_ros2::ModeBase::kModeIDPosctl);
    command.source_component =
      px4_msgs::msg::VehicleCommand::COMPONENT_MODE_EXECUTOR_START + executor_.id();
    command.timestamp = 0;
    command_publisher_->publish(command);
    last_request_time_ = now;
    ++attempts_;
  }

  FlyingHandModeExecutor & executor_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  bool requested_{false};
  bool failure_reported_{false};
  int attempts_{0};
  SteadyTime last_request_time_{};
};

FlyingHandModeExecutor::FlyingHandModeExecutor(
  FlyingHandMode & mode, const std::string & px4_topic_namespace_prefix)
: ModeExecutorBase(ModeExecutorBase::Settings{}, mode), mode_(mode),
  impl_(std::make_unique<Impl>(*this, px4_topic_namespace_prefix))
{
  mode_.setFallbackCallback(
    [this](FaultReason reason) {requestPositionFallback(reason);});
}

FlyingHandModeExecutor::~FlyingHandModeExecutor()
{
  mode_.setFallbackCallback({});
}

void FlyingHandModeExecutor::onActivate()
{
  impl_->activate();
}

void FlyingHandModeExecutor::onDeactivate(DeactivateReason reason)
{
  (void)reason;
  impl_->deactivate();
}

void FlyingHandModeExecutor::requestPositionFallback(FaultReason reason)
{
  impl_->request(reason);
}

}  // namespace flying_hand_mode::runtime
