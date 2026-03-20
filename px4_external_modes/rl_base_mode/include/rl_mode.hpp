#pragma once

#include <px4_ros2/components/mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <inference/infer_backend.hpp>
#include <robot_data.hpp>
#include <rnn_state_manager.hpp>
#include <flight_logger.hpp>

/**
 * @class RLModeBase
 * @brief A base class for reinforcement learning-based flight modes.
 */
class RLModeBase : public px4_ros2::ModeBase
{
public:
  /**
   * @brief Constructor for RLModeBase.
   * @param node The rclcpp::Node for this mode.
   * @param backend The inference backend for the policy network.
   * @param settings The settings for the ModeBase.
   * @param topic_namespace_prefix The namespace prefix for PX4 topics.
   */
  RLModeBase(
    rclcpp::Node & node, std::unique_ptr<InferBackend> backend,
    px4_ros2::ModeBase::Settings settings, const std::string & topic_namespace_prefix = "",
    const std::string & root_dir = "");

  /// @brief Virtual destructor.
  virtual ~RLModeBase() = default;

  /**
   * @brief Pure virtual function to collect observations for the policy network.
   * @param[out] inputs The tensor map to be filled with observation data.
   * @param dt_s The time delta in seconds since the last update.
   */
  virtual void getObservation(TensorMap & inputs, float dt_s) = 0;

  /**
   * @brief Pure virtual function to apply the actions from the policy network.
   * @param action The output tensor map from the model.
   * @param dt_s The time delta in seconds since the last update.
   */
  virtual void applyAction(const TensorMap & action, float dt_s) = 0;

  /**
   * @brief Overrides the base method to implement the RL control loop.
   * This method calls getObservation, forward, and applyAction in sequence.
   * @param dt_s The time delta in seconds since the last setpoint update.
   */
  void updateSetpoint(float dt_s) override;

  /**
   * @brief Provides access to the robot's state data.
   * @return A pointer to the RobotData instance.
   */
  RobotData * robotData();

  /**
   * @brief Type-safe accessor for extended RobotData implementations.
   * @tparam T Target RobotData-derived type.
   * @return Converted pointer, or nullptr if the runtime type does not match.
   */
  template<typename T>
  T * robotDataAs()
  {
    return dynamic_cast<T *>(robot_data_.get());
  }

  /**
   * @brief Const-qualified type-safe accessor for extended RobotData types.
   * @tparam T Target RobotData-derived type.
   * @return Converted pointer, or nullptr if the runtime type does not match.
   */
  template<typename T>
  const T * robotDataAs() const
  {
    return dynamic_cast<const T *>(robot_data_.get());
  }

  /**
   * @brief Provides access to the flight logger.
   * @return A pointer to the FlightLogger instance.
   */
  FlightLogger * flightLogger();

protected:
  /// @brief Accesses recurrent-state manager used for optional `h_in`/`h_out` handling.
  RnnStateManager & rnnState();
  /// @brief Const overload of recurrent-state manager accessor.
  const RnnStateManager & rnnState() const;

  /**
   * @brief Replaces the active robot-data provider.
   * @param robot_data New robot-data provider owned by this mode.
   */
  void setRobotData(std::unique_ptr<RobotData> robot_data);

  /**
   * @brief Logs current odometry data to the flight logger.
   */
  void logOdometryData();

  /**
   * @brief Logs observation tensors to the flight logger.
   * @param observations The observation tensor map to log.
   * @param dt_s The time delta in seconds since the last update.
   */
  void logObservations(const TensorMap & observations, float dt_s);

  /**
   * @brief Logs action tensors to the flight logger.
   * @param actions The action tensor map to log.
   * @param dt_s The time delta in seconds since the last update.
   */
  void logActions(const TensorMap & actions, float dt_s);

  /**
   * @brief Logs observation or action tensors to the flight logger.
   * @param topic The topic name (e.g., "observations", "actions").
   * @param tensor_map The tensor map to log.
   * @param now_s The current time in seconds.
   */
  void logTensor(const std::string & topic, const TensorMap & tensor_map, double now_s);

private:
  /// @brief The inference backend for the policy network.
  std::unique_ptr<InferBackend> backend_;

  /// @brief The robot state data provider.
  std::unique_ptr<RobotData> robot_data_;

  /// @brief The flight logger.
  std::unique_ptr<FlightLogger> flight_logger_;

  RnnStateManager rnn_state_;  ///< Recurrent state helper shared by all derived RL modes.

protected:
  /// @brief Accessor for derived classes to query backend properties (e.g., input shapes).
  InferBackend * backend();
  const InferBackend * backend() const;
};
