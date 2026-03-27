#include <functional>
#include <memory>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "common.hpp"
#include "px4_ros2/utils/message_version.hpp"

class OdometryFromPX4 : public rclcpp::Node
{
public:
  OdometryFromPX4()
  : Node("odom_from_px4")
  {
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Subscribe to PX4 raw odometry messages
    px4_odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleOdometry>(),
      qos,
      std::bind(&OdometryFromPX4::px4OdomCallback, this, std::placeholders::_1));

    // Configure sensor parameters
    this->declare_parameter<std::string>("sensor_name", "sensor");
    this->declare_parameter<std::string>("odom_frame", "world");
    this->declare_parameter<std::string>("sensor_frame", "sensor_link");
    this->declare_parameter<std::string>("odom_topic", "");

    // Get sensor name and configure topic
    sensor_name_ = this->get_parameter("sensor_name").as_string();
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    if (odom_topic.empty()) {
      odom_topic = "/" + sensor_name_ + "/odom";
    }

    // Publish sensor pose odometry messages
    ros_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    // Initialize tf2 broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Declare sensor extrinsic parameters (relative to drone body frame)
    this->declare_parameter<std::vector<double>>("sensor_translation", {0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>(
      "sensor_rotation_matrix", {1.0, 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 1.0});  // 3x3 rotation matrix [r11, ..., r33]

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
      // Default to identity matrix if wrong size
      sensor_rotation_matrix_ = Eigen::Matrix3d::Identity();
      RCLCPP_WARN(get_logger(), "Invalid rotation matrix size. Using identity matrix.");
    }

    RCLCPP_INFO(get_logger(), "PX4 %s Odometry Converter started", sensor_name_.c_str());
    RCLCPP_INFO(
      get_logger(), "Sensor translation: [%.3f, %.3f, %.3f]",
      sensor_translation_.x(), sensor_translation_.y(), sensor_translation_.z());
    RCLCPP_INFO(
      get_logger(), "Sensor rotation matrix:\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f",
      sensor_rotation_matrix_(0, 0), sensor_rotation_matrix_(0, 1), sensor_rotation_matrix_(0, 2),
      sensor_rotation_matrix_(1, 0), sensor_rotation_matrix_(1, 1), sensor_rotation_matrix_(1, 2),
      sensor_rotation_matrix_(2, 0), sensor_rotation_matrix_(2, 1), sensor_rotation_matrix_(2, 2));
    RCLCPP_INFO(get_logger(), "Publishing odometry to topic: %s", odom_topic.c_str());
  }

private:
  // Convert PX4 NED coordinates to ENU coordinates for sensor pose
  void px4OdomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // Create ROS standard Odometry message
    auto odom_msg = nav_msgs::msg::Odometry();

    // Set header
    odom_msg.header = timestamp_to_header(msg->timestamp);
    odom_msg.header.frame_id = this->get_parameter("odom_frame").as_string();
    odom_msg.child_frame_id = this->get_parameter("sensor_frame").as_string();

    // 1. Get drone body pose in ENU coordinate system
    // Position: PX4 odometry position is "NED" so needs to be converted to "ENU"
    const Eigen::Vector3d body_position_enu(
      msg->position[1], msg->position[0], -msg->position[2]);

    // Attitude: PX4 odometry attitude is "body FRD->NED" so needs to be converted to "body FLU->ENU"
    const Eigen::Quaterniond body_orientation_enu = px4_ros2::attitudeNedToEnu(
      Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]));

    // 2. Calculate sensor pose in ENU coordinate system
    // sensor position = body position + body attitude rotated sensor translation
    const Eigen::Vector3d sensor_position_enu = body_position_enu + body_orientation_enu *
      sensor_translation_;

    // sensor attitude = body attitude * sensor relative attitude (using rotation matrix)
    const Eigen::Matrix3d body_rotation_matrix = body_orientation_enu.toRotationMatrix();
    const Eigen::Matrix3d sensor_rotation_matrix_enu =
      body_rotation_matrix * sensor_rotation_matrix_;
    const Eigen::Quaterniond sensor_orientation_enu(sensor_rotation_matrix_enu);

    // Set sensor pose
    odom_msg.pose.pose.position.x = sensor_position_enu.x();
    odom_msg.pose.pose.position.y = sensor_position_enu.y();
    odom_msg.pose.pose.position.z = sensor_position_enu.z();

    odom_msg.pose.pose.orientation.x = sensor_orientation_enu.x();
    odom_msg.pose.pose.orientation.y = sensor_orientation_enu.y();
    odom_msg.pose.pose.orientation.z = sensor_orientation_enu.z();
    odom_msg.pose.pose.orientation.w = sensor_orientation_enu.w();

    // 3. Calculate sensor velocity (considering rotation-induced velocity)
    // Body linear velocity: PX4 odometry linear velocity is "NED" so needs to be converted to "ENU"
    const Eigen::Vector3d body_linear_vel_enu(
      msg->velocity[1], msg->velocity[0], -msg->velocity[2]);

    // Body angular velocity: PX4 odometry angular velocity is "body FRD" so needs to be converted to "body FLU"
    const Eigen::Vector3d body_angular_vel_flu(
      msg->angular_velocity[0], -msg->angular_velocity[1], -msg->angular_velocity[2]);

    // sensor linear velocity = body linear velocity + body angular velocity × sensor translation offset
    const Eigen::Vector3d sensor_linear_vel_enu = body_linear_vel_enu + body_angular_vel_flu.cross(
      body_orientation_enu * sensor_translation_);

    // sensor angular velocity = body angular velocity (assuming sensor frame aligns with body frame, or rotation in extrinsic doesn't produce additional angular velocity)
    Eigen::Vector3d sensor_angular_vel_flu = body_angular_vel_flu;

    odom_msg.twist.twist.linear.x = sensor_linear_vel_enu.x();
    odom_msg.twist.twist.linear.y = sensor_linear_vel_enu.y();
    odom_msg.twist.twist.linear.z = sensor_linear_vel_enu.z();

    odom_msg.twist.twist.angular.x = sensor_angular_vel_flu.x();
    odom_msg.twist.twist.angular.y = sensor_angular_vel_flu.y();
    odom_msg.twist.twist.angular.z = sensor_angular_vel_flu.z();

    // Covariance matrix (can be adjusted as needed)
    // Position covariance
    odom_msg.pose.covariance[0] = msg->position_variance[1];
    odom_msg.pose.covariance[7] = msg->position_variance[0];
    odom_msg.pose.covariance[14] = msg->position_variance[2];
    // Attitude covariance
    odom_msg.pose.covariance[21] = msg->orientation_variance[0];  // roll
    odom_msg.pose.covariance[28] = msg->orientation_variance[1];  // pitch
    odom_msg.pose.covariance[35] = msg->orientation_variance[2];  // yaw

    // Velocity covariance
    odom_msg.twist.covariance[0] = msg->velocity_variance[1];
    odom_msg.twist.covariance[7] = msg->velocity_variance[0];
    odom_msg.twist.covariance[14] = msg->velocity_variance[2];

    // Publish sensor pose
    ros_odom_pub_->publish(odom_msg);

    // 4. Publish tf2 transform
    publishTransform(odom_msg.header.stamp, sensor_position_enu, sensor_orientation_enu);
  }

  /**
   * @brief Publish tf2 transform from odom_frame to sensor_frame
   */
  void publishTransform(
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & position,
    const Eigen::Quaterniond & orientation)
  {
    geometry_msgs::msg::TransformStamped transform;

    transform.header.stamp = stamp;
    transform.header.frame_id = this->get_parameter("odom_frame").as_string();
    transform.child_frame_id = this->get_parameter("sensor_frame").as_string();

    // Set translation
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();

    // Set rotation
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();

    // Publish transform
    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr px4_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Sensor extrinsic parameters
  Eigen::Vector3d sensor_translation_;      // Sensor translation relative to body
  Eigen::Matrix3d sensor_rotation_matrix_;  // Sensor rotation matrix relative to body
  std::string sensor_name_;                 // Sensor name for logging
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryFromPX4>());
  rclcpp::shutdown();
  return 0;
}
