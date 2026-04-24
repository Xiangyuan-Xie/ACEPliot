#include <trajectory_generator_utils/state_provider.hpp>

#include <cmath>

#include <px4_ros2/utils/frame_conversion.hpp>

TrajectoryStateProvider::TrajectoryStateProvider(
  rclcpp::Node & node,
  px4_ros2::Context & px4_context,
  const StateProviderConfig & config)
: config_(config)
{
  if (config_.use_ros2_odom) {
    odom_sub_ = node.create_subscription<nav_msgs::msg::Odometry>(
      config_.odom_topic,
      10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_msg_ = *msg;
        has_ros2_odom_ = true;
      });
    return;
  }

  local_pos_ = std::make_shared<px4_ros2::OdometryLocalPosition>(px4_context);
  attitude_ = std::make_shared<px4_ros2::OdometryAttitude>(px4_context);
  angular_vel_ = std::make_shared<px4_ros2::OdometryAngularVelocity>(px4_context);
}

bool TrajectoryStateProvider::getState(TrajectoryGeneratorState & state) const
{
  if (config_.use_ros2_odom) {
    if (!has_ros2_odom_) {
      return false;
    }

    state.root_pos_w = Eigen::Vector3f(
      static_cast<float>(last_odom_msg_.pose.pose.position.x),
      static_cast<float>(last_odom_msg_.pose.pose.position.y),
      static_cast<float>(last_odom_msg_.pose.pose.position.z));
    state.root_quat_w = Eigen::Quaternionf(
      static_cast<float>(last_odom_msg_.pose.pose.orientation.w),
      static_cast<float>(last_odom_msg_.pose.pose.orientation.x),
      static_cast<float>(last_odom_msg_.pose.pose.orientation.y),
      static_cast<float>(last_odom_msg_.pose.pose.orientation.z));
    state.root_lin_vel_b = Eigen::Vector3f(
      static_cast<float>(last_odom_msg_.twist.twist.linear.x),
      static_cast<float>(last_odom_msg_.twist.twist.linear.y),
      static_cast<float>(last_odom_msg_.twist.twist.linear.z));

    const float w = state.root_quat_w.w();
    const float x = state.root_quat_w.x();
    const float y = state.root_quat_w.y();
    const float z = state.root_quat_w.z();
    state.heading_w = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    return true;
  }

  if (!local_pos_->lastValid() || !attitude_->lastValid() || !angular_vel_->lastValid()) {
    return false;
  }

  state.root_pos_w = px4_ros2::positionNedToEnu(local_pos_->positionNed());
  state.root_quat_w = px4_ros2::attitudeNedToEnu(attitude_->attitude());
  state.root_lin_vel_b =
    state.root_quat_w.inverse() * px4_ros2::positionNedToEnu(local_pos_->velocityNed());
  state.heading_w = px4_ros2::yawNedToEnu(local_pos_->heading());
  return true;
}
