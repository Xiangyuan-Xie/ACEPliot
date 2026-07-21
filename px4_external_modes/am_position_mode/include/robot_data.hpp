#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <memory>
#include <string>
#include <nav_msgs/msg/odometry.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/angular_velocity.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>

using namespace std::chrono_literals;

/**
 * @class RobotData
 * @brief Unified accessor for vehicle state and frame conversion.
 *
 * This class supports two state sources:
 * 1) Native PX4 odometry components;
 * 2) External ROS2 Odometry subscription.
 */
class RobotData
{
public:
  /**
   * @brief Constructs the state accessor and initializes data sources.
   * @param mode_base Current flight mode object used to access node and PX4 components.
   */
  explicit RobotData(px4_ros2::ModeBase & mode_base);

  /// @brief Default destructor.
  virtual ~RobotData() = default;

  /// @brief Updates internal state cache from the active data source.
  virtual void updateState();

  /// @brief Returns vehicle position in world frame (ENU).
  const Eigen::Vector3f & RootPosW() const;

  /// @brief Returns vehicle orientation quaternion in world frame (ENU).
  const Eigen::Quaternionf & RootQuatW() const;

  /// @brief Returns vehicle linear velocity in world frame (ENU).
  const Eigen::Vector3f & RootLinVelW() const;

  /// @brief Returns vehicle angular velocity in world frame (ENU).
  const Eigen::Vector3f & RootAngVelW() const;

  /// @brief Returns vehicle linear velocity in body frame (FLU).
  const Eigen::Vector3f & RootLinVelB() const;

  /// @brief Returns vehicle angular velocity in body frame (FLU).
  const Eigen::Vector3f & RootAngVelB() const;

  /// @brief Returns gravity projected into body frame.
  const Eigen::Vector3f & ProjectedGravityB() const;

  /// @brief Returns heading angle in world frame (ENU yaw).
  float HeadingW() const;

protected:
  /// @brief Refreshes cached state from native PX4 odometry components.
  void updatePX4State();

  /// @brief Callback for external ROS2 odometry input.
  /// @param msg External odometry message.
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  px4_ros2::ModeBase * mode_base_{nullptr};  ///< Owning mode object.

private:
  std::shared_ptr<px4_ros2::OdometryLocalPosition> local_pos_;      ///< PX4 position/velocity source.
  std::shared_ptr<px4_ros2::OdometryAttitude> attitude_;            ///< PX4 attitude source.
  std::shared_ptr<px4_ros2::OdometryAngularVelocity> angular_vel_;  ///< PX4 angular velocity source.
  bool use_ros2_odom_{false};                                       ///< True when using external ROS2 odometry.
  std::string odom_topic_;                                          ///< Subscribed external odometry topic.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;  ///< External odometry subscription.
  Eigen::Vector3f root_pos_w_;                                      ///< Position in world frame (ENU).
  Eigen::Quaternionf root_quat_w_;                                  ///< Orientation in world frame (ENU).
  Eigen::Vector3f root_lin_vel_w_;                                  ///< Linear velocity in world frame (ENU).
  Eigen::Vector3f root_ang_vel_w_;                                  ///< Angular velocity in world frame (ENU).
  float heading_w_{0.0f};                                           ///< Heading in world frame (ENU yaw).
  Eigen::Vector3f root_lin_vel_b_;                                  ///< Linear velocity in body frame (FLU).
  Eigen::Vector3f root_ang_vel_b_;                                  ///< Angular velocity in body frame (FLU).
  Eigen::Vector3f projected_gravity_b_;                             ///< Gravity projection in body frame.
};
