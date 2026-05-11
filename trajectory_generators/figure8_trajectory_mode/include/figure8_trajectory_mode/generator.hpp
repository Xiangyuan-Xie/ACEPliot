#pragma once
#include <Eigen/Eigen>
#include <optional>

/**
 * @struct TrajectorySample
 * @brief Single-step output from a trajectory generator.
 *
 * Each field is optional, allowing callers to consume only the required subset.
 */
struct TrajectorySample
{
  std::optional<Eigen::Vector3f> position;      ///< Desired position in world frame.
  std::optional<Eigen::Vector3f> velocity;      ///< Desired velocity in world frame.
  std::optional<Eigen::Vector3f> acceleration;  ///< Desired acceleration in world frame.
  std::optional<float> yaw, yaw_rate;           ///< Desired yaw and yaw rate.
};

/**
 * @struct TrajectoryGeneratorState
 * @brief Minimal state snapshot required by generator strategies.
 */
struct TrajectoryGeneratorState
{
  Eigen::Vector3f root_pos_w{Eigen::Vector3f::Zero()};
  Eigen::Quaternionf root_quat_w{Eigen::Quaternionf::Identity()};
  Eigen::Vector3f root_lin_vel_b{Eigen::Vector3f::Zero()};
  float heading_w{0.0f};
};

/**
 * @class ITrajectoryGenerator
 * @brief Unified interface for trajectory generators.
 */
class ITrajectoryGenerator
{
public:
  /// @brief Virtual destructor.
  virtual ~ITrajectoryGenerator() = default;

  /**
   * @brief Resets internal generator state.
   * @param state Current vehicle state snapshot.
   */
  virtual void reset(const TrajectoryGeneratorState & state) = 0;

  /**
   * @brief Advances one step and produces trajectory commands.
   * @param dt_s Control cycle duration in seconds.
   * @param state Current vehicle state snapshot.
   * @return Trajectory output for the current step.
   */
  virtual TrajectorySample step(float dt_s, const TrajectoryGeneratorState & state) = 0;
};
