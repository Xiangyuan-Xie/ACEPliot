#pragma once

#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/odometry/angular_velocity.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>

#include <figure8_trajectory_mode/generator.hpp>

struct StateProviderConfig
{
  bool use_ros2_odom{false};
  std::string odom_topic{"/odom"};
};

class TrajectoryStateProvider
{
public:
  TrajectoryStateProvider(
    rclcpp::Node & node,
    px4_ros2::Context & px4_context,
    const StateProviderConfig & config);

  bool getState(TrajectoryGeneratorState & state) const;

private:
  StateProviderConfig config_;
  nav_msgs::msg::Odometry last_odom_msg_{};
  bool has_ros2_odom_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> local_pos_;
  std::shared_ptr<px4_ros2::OdometryAttitude> attitude_;
  std::shared_ptr<px4_ros2::OdometryAngularVelocity> angular_vel_;
};
