/**
 * @file gt_odom_from_px4_node.cpp
 * @brief A ROS2 node that subscribes to PX4 ground truth odometry and publishes sensor-specific odometry with extrinsic calibration.
 */

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include "px4_ros2/utils/message_version.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "common.hpp"

class GTOdometryFromPX4 : public rclcpp::Node
{
public:
  GTOdometryFromPX4()
  : Node("gt_odom_from_px4")
  {
    // Configure sensor parameters
    this->declare_parameter<std::string>("sensor_name", "sensor");
    this->declare_parameter<std::string>("odom_frame", "world");
    this->declare_parameter<std::string>("sensor_frame", "sensor_link");
    this->declare_parameter<std::string>("odom_topic", "");

    // Get sensor name and configure topic
    sensor_name_ = this->get_parameter("sensor_name").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    if (odom_topic.empty()) {
      odom_topic = "/" + sensor_name_ + "/gt_odom";
    }

    // Publish sensor ground truth odometry messages
    gt_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    // Initialize tf2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Declare sensor extrinsic parameters (relative to drone body frame)
    this->declare_parameter<std::vector<double>>("sensor_translation", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>(
      "sensor_rotation_matrix", {1.0, 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 1.0});

    // Get sensor extrinsic parameters
    auto trans_vec = this->get_parameter("sensor_translation").as_double_array();
    auto rot_matrix_vec = this->get_parameter("sensor_rotation_matrix").as_double_array();

    // Set sensor transform relative to body
    sensor_translation_ << trans_vec[0], trans_vec[1], trans_vec[2];

    // Convert rotation matrix vector to Eigen::Matrix3d
    if (rot_matrix_vec.size() == 9) {
      sensor_rotation_matrix_ << rot_matrix_vec[0], rot_matrix_vec[1], rot_matrix_vec[2],
        rot_matrix_vec[3], rot_matrix_vec[4], rot_matrix_vec[5],
        rot_matrix_vec[6], rot_matrix_vec[7], rot_matrix_vec[8];
    } else {
      sensor_rotation_matrix_ = Eigen::Matrix3d::Identity();
      RCLCPP_WARN(get_logger(), "Invalid rotation matrix size. Using identity matrix.");
    }

    // Set up synchronized subscribers for ground truth data
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sub1_ = std::make_shared<message_filters::Subscriber<px4_msgs::msg::VehicleLocalPosition>>(
      this,
      "/fmu/out/vehicle_local_position_groundtruth" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(),
      qos.get_rmw_qos_profile());

    sub2_ = std::make_shared<message_filters::Subscriber<px4_msgs::msg::VehicleAttitude>>(
      this,
      "/fmu/out/vehicle_attitude_groundtruth" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleAttitude>(),
      qos.get_rmw_qos_profile());

    sync_msg_ = std::make_shared<message_filters::TimeSynchronizer<
          px4_msgs::msg::VehicleLocalPosition, px4_msgs::msg::VehicleAttitude>>(*sub1_, *sub2_, 10);

    sync_msg_->registerCallback(
      std::bind(&GTOdometryFromPX4::callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(), "PX4 Ground Truth %s Odometry Converter started",
      sensor_name_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing ground truth odometry to topic: %s", odom_topic.c_str());
  }

private:
  void callback(
    const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr & local_pos,
    const px4_msgs::msg::VehicleAttitude::ConstSharedPtr & att)
  {
    auto odom_msg = nav_msgs::msg::Odometry();

    odom_msg.header = timestamp_to_header(local_pos->timestamp);
    odom_msg.header.frame_id = this->get_parameter("odom_frame").as_string();
    odom_msg.child_frame_id = this->get_parameter("sensor_frame").as_string();

    // Convert NED to ENU coordinates
    Eigen::Vector3d body_position_enu;
    body_position_enu << local_pos->y, local_pos->x, -local_pos->z;

    Eigen::Quaterniond body_orientation_enu = px4_ros2::attitudeNedToEnu(
      Eigen::Quaterniond(att->q[0], att->q[1], att->q[2], att->q[3]));

    // Apply sensor extrinsic transformation
    Eigen::Vector3d sensor_position_enu = body_position_enu + body_orientation_enu *
      sensor_translation_;

    Eigen::Matrix3d body_rotation_matrix = body_orientation_enu.toRotationMatrix();
    Eigen::Matrix3d sensor_rotation_matrix_enu = body_rotation_matrix * sensor_rotation_matrix_;
    Eigen::Quaterniond sensor_orientation_enu(sensor_rotation_matrix_enu);

    // Set pose
    odom_msg.pose.pose.position.x = sensor_position_enu.x();
    odom_msg.pose.pose.position.y = sensor_position_enu.y();
    odom_msg.pose.pose.position.z = sensor_position_enu.z();

    odom_msg.pose.pose.orientation.x = sensor_orientation_enu.x();
    odom_msg.pose.pose.orientation.y = sensor_orientation_enu.y();
    odom_msg.pose.pose.orientation.z = sensor_orientation_enu.z();
    odom_msg.pose.pose.orientation.w = sensor_orientation_enu.w();

    // Calculate velocity (using local position velocity if available, otherwise set to zero)
    Eigen::Vector3d body_linear_vel_enu;
    if (local_pos->v_xy_valid && local_pos->v_z_valid) {
      body_linear_vel_enu << local_pos->vy, local_pos->vx, -local_pos->vz;
    } else {
      body_linear_vel_enu << 0.0, 0.0, 0.0;
    }

    // Angular velocity from attitude (if available)
    Eigen::Vector3d body_angular_vel_flu;
    if (att->timestamp != 0) {
      // For ground truth, we might not have angular velocity, so set to zero
      body_angular_vel_flu << 0.0, 0.0, 0.0;
    } else {
      body_angular_vel_flu << 0.0, 0.0, 0.0;
    }

    Eigen::Vector3d sensor_linear_vel_enu = body_linear_vel_enu + body_angular_vel_flu.cross(
      body_orientation_enu * sensor_translation_);
    Eigen::Vector3d sensor_angular_vel_flu = body_angular_vel_flu;

    odom_msg.twist.twist.linear.x = sensor_linear_vel_enu.x();
    odom_msg.twist.twist.linear.y = sensor_linear_vel_enu.y();
    odom_msg.twist.twist.linear.z = sensor_linear_vel_enu.z();

    odom_msg.twist.twist.angular.x = sensor_angular_vel_flu.x();
    odom_msg.twist.twist.angular.y = sensor_angular_vel_flu.y();
    odom_msg.twist.twist.angular.z = sensor_angular_vel_flu.z();

    // Set ground truth covariance (very low uncertainty)
    for (int i = 0; i < 36; ++i) {
      odom_msg.pose.covariance[i] = 1e-9;
      odom_msg.twist.covariance[i] = 1e-9;
    }

    gt_odom_pub_->publish(odom_msg);
    publishTransform(odom_msg.header.stamp, sensor_position_enu, sensor_orientation_enu);
  }

  void publishTransform(
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation)
  {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = stamp;
    transform.header.frame_id = this->get_parameter("odom_frame").as_string();
    transform.child_frame_id = this->get_parameter("sensor_frame").as_string();

    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();

    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();

    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr gt_odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<message_filters::Subscriber<px4_msgs::msg::VehicleLocalPosition>> sub1_;
  std::shared_ptr<message_filters::Subscriber<px4_msgs::msg::VehicleAttitude>> sub2_;
  std::shared_ptr<message_filters::TimeSynchronizer<
      px4_msgs::msg::VehicleLocalPosition, px4_msgs::msg::VehicleAttitude>> sync_msg_;

  Eigen::Vector3d sensor_translation_;
  Eigen::Matrix3d sensor_rotation_matrix_;
  std::string sensor_name_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GTOdometryFromPX4>());
  rclcpp::shutdown();
  return 0;
}
