#pragma once
#include <Eigen/Eigen>
#include <optional>

class RobotData;

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
   * @param robot_data Current vehicle state snapshot.
   */
  virtual void reset(const RobotData & robot_data) = 0;

  /**
   * @brief Advances one step and produces trajectory commands.
   * @param dt_s Control cycle duration in seconds.
   * @param robot_data Current vehicle state snapshot.
   * @return Trajectory output for the current step.
   */
  virtual TrajectorySample step(float dt_s, const RobotData & robot_data) = 0;
};
