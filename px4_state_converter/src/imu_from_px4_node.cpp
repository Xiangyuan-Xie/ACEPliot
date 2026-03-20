/**
 * @file px4_imu_converter.cpp
 * @brief Converts PX4 SensorCombined IMU messages (FRD frame) to standard ROS2 IMU messages (FLU frame).
 */

 #include <rclcpp/rclcpp.hpp>
 #include <px4_msgs/msg/sensor_combined.hpp>
 #include <px4_ros2/utils/frame_conversion.hpp>
 #include "px4_ros2/utils/message_version.hpp"
 #include <sensor_msgs/msg/imu.hpp>
 #include <Eigen/Geometry>
#include "common.hpp"
/**
  * @class ImuFromPX4
  * @brief A ROS2 node that converts PX4 raw IMU messages to standard sensor_msgs::msg::Imu messages.
  */
class ImuFromPX4 : public rclcpp::Node
{
public:
  /**
   * @brief Constructor: Initializes subscribers and publishers.
   */
  ImuFromPX4()
  : Node("imu_from_px4")
  {
    // Use sensor data QoS profile
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    // Subscribe to PX4 raw IMU topic
    px4_imu_sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
      "/fmu/out/sensor_combined" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::SensorCombined>(),
      qos,
      std::bind(&ImuFromPX4::px4ImuCallback, this, std::placeholders::_1));

    // Publish standard ROS2 IMU topic
    ros_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/px4/imu", 10);
  }

private:
  /**
   * @brief Callback to convert and publish IMU data in standard ROS2 format (FLU).
   * @param msg PX4 IMU message (in FRD frame)
   */
  void px4ImuCallback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
  {
    sensor_msgs::msg::Imu imu_msg;

    // Set timestamp from PX4 time
    imu_msg.header = timestamp_to_header(msg->timestamp);
    imu_msg.header.frame_id = "imu_frame";

    // Convert linear acceleration from FRD to FLU
    Eigen::Vector3d accel_frd(
      msg->accelerometer_m_s2[0],   // X: Forward
      msg->accelerometer_m_s2[1],   // Y: Right
      msg->accelerometer_m_s2[2]    // Z: Down
    );
    Eigen::Vector3d accel_flu = px4_ros2::frdToFlu(accel_frd);
    imu_msg.linear_acceleration.x = accel_flu.x();
    imu_msg.linear_acceleration.y = accel_flu.y();
    imu_msg.linear_acceleration.z = accel_flu.z();

    // Convert angular velocity from FRD to FLU
    Eigen::Vector3d gyro_frd(
      msg->gyro_rad[0],   // X: roll rate
      msg->gyro_rad[1],   // Y: pitch rate
      msg->gyro_rad[2]    // Z: yaw rate
    );
    Eigen::Vector3d gyro_flu = px4_ros2::frdToFlu(gyro_frd);
    imu_msg.angular_velocity.x = gyro_flu.x();
    imu_msg.angular_velocity.y = gyro_flu.y();
    imu_msg.angular_velocity.z = gyro_flu.z();

    // Covariance settings:
    // These are set to -1 indicating "unknown" or "not provided"
    imu_msg.linear_acceleration_covariance[0] = -1.0;
    imu_msg.angular_velocity_covariance[0] = -1.0;
    imu_msg.orientation_covariance[0] = -1.0;   // Orientation not provided

    // Publish the converted IMU message
    ros_imu_pub_->publish(imu_msg);
  }

  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_imu_sub_;   /**< PX4 IMU subscriber */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr ros_imu_pub_;              /**< Standard ROS IMU publisher */
};

/**
  * @brief Main function for the PX4 IMU Converter node.
  */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuFromPX4>());
  rclcpp::shutdown();
  return 0;
}
