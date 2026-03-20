/**
* @file ground_truth_odometry.cpp
* @brief A ROS2 node that synchronizes PX4 ground truth local position and attitude, and publishes fused pose to PX4 navigation interface.
*/

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include "px4_ros2/utils/message_version.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace std::chrono_literals;

static const std::string topic_namespace_prefix = "";

/**
* @class LocalNavigation
* @brief Wraps PX4 local position measurement interface for ground truth pose injection.
*/
class LocalNavigation : public px4_ros2::LocalPositionMeasurementInterface
{
public:
  /**
  * @brief Constructor
  * @param node The ROS2 node handle
  */
  explicit LocalNavigation(rclcpp::Node & node)
  : LocalPositionMeasurementInterface(node, px4_ros2::PoseFrame::LocalNED,
      px4_ros2::VelocityFrame::LocalNED) {}

  //  px4_ros2::VelocityFrame::LocalNED, topic_namespace_prefix) {}

  /**
  * @brief Sends a PoseStamped message to PX4 local position interface.
  * @param msg PoseStamped input message
  */
  void updateLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    px4_ros2::LocalPositionMeasurement local_position_measurement {};
    local_position_measurement.timestamp_sample = msg->header.stamp;

    local_position_measurement.position_xy = Eigen::Vector2f {
      msg->pose.position.x, msg->pose.position.y
    };
    local_position_measurement.position_xy_variance = Eigen::Vector2f {1e-9f, 1e-9f};

    local_position_measurement.position_z = msg->pose.position.z;
    local_position_measurement.position_z_variance = 1e-9f;

    local_position_measurement.velocity_xy = Eigen::Vector2f {0.0f, 0.0f};
    local_position_measurement.velocity_xy_variance = Eigen::Vector2f {1e-9f, 1e-9f};

    local_position_measurement.velocity_z = 0.0f;
    local_position_measurement.velocity_z_variance = 1e-9f;

    local_position_measurement.attitude_quaternion = Eigen::Quaternionf {
      static_cast<float>(msg->pose.orientation.w),
      static_cast<float>(msg->pose.orientation.x),
      static_cast<float>(msg->pose.orientation.y),
      static_cast<float>(msg->pose.orientation.z)
    };
    local_position_measurement.attitude_variance = Eigen::Vector3f {0.001f, 0.001f, 0.001f};

    try {
      update(local_position_measurement);
      RCLCPP_DEBUG(
        _node.get_logger(),
        "Successfully sent position update to navigation interface.");
    } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
      RCLCPP_ERROR_THROTTLE(
        _node.get_logger(),
        *_node.get_clock(), 1000, "Exception caught: %s", e.what());
    }
  }
};

/**
* @class GroundTruthOdometry
* @brief ROS2 node that fuses PX4 local position and attitude ground truth into PoseStamped and updates PX4 navigation.
*/
class GroundTruthOdometry : public rclcpp::Node
{
public:
  /**
  * @brief Constructor. Sets up subscriptions and synchronization.
  */
  GroundTruthOdometry()
  : Node("ground_truth_odometry_node")
  {
    _interface = std::make_unique<LocalNavigation>(*this);

    if (!_interface->doRegister()) {
      throw std::runtime_error("Registration failed");
    }

    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sub1_ = std::make_shared<message_filters::Subscriber<px4_msgs::msg::VehicleLocalPosition>>(
      this,
      topic_namespace_prefix + "/fmu/out/vehicle_local_position_groundtruth" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleLocalPosition>(),
      qos.get_rmw_qos_profile());

    sub2_ = std::make_shared<message_filters::Subscriber<px4_msgs::msg::VehicleAttitude>>(
      this,
      topic_namespace_prefix + "/fmu/out/vehicle_attitude_groundtruth" +
      px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleAttitude>(),
      qos.get_rmw_qos_profile());

    sync_msg_ = std::make_shared<message_filters::TimeSynchronizer<
          px4_msgs::msg::VehicleLocalPosition, px4_msgs::msg::VehicleAttitude>>(*sub1_, *sub2_, 10);

    sync_msg_->registerCallback(
      std::bind(
        &GroundTruthOdometry::callback, this, std::placeholders::_1,
        std::placeholders::_2));
  }

  /**
  * @brief Callback function when both local position and attitude messages are synchronized.
  * @param local_pos The vehicle local position message
  * @param att The vehicle attitude message
  */
  void callback(
    const px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr & local_pos,
    const px4_msgs::msg::VehicleAttitude::ConstSharedPtr & att)
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = timestamp_to_header(local_pos->timestamp);

    pose_msg.pose.position.x = local_pos->x;
    pose_msg.pose.position.y = local_pos->y;
    pose_msg.pose.position.z = local_pos->z;

    pose_msg.pose.orientation.x = att->q[1];
    pose_msg.pose.orientation.y = att->q[2];
    pose_msg.pose.orientation.z = att->q[3];
    pose_msg.pose.orientation.w = att->q[0];

    auto pose_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>(pose_msg);
    _interface->updateLocalPosition(pose_ptr);
  }

  /**
  * @brief Converts PX4 timestamp (in microseconds) to ROS2 header timestamp.
  * @param timestamp PX4 timestamp in microseconds
  * @return ROS2 standard message header with converted time
  */
  std_msgs::msg::Header timestamp_to_header(uint64_t timestamp)
  {
    std_msgs::msg::Header header;
    header.stamp.sec = static_cast<int32_t>(timestamp / 1'000'000);  // seconds
    header.stamp.nanosec = static_cast<uint32_t>((timestamp % 1'000'000) * 1000);  // remaining microseconds to ns
    return header;
  }

private:
  std::unique_ptr<LocalNavigation> _interface;  /**< PX4 navigation interface */

  std::shared_ptr<message_filters::Subscriber<px4_msgs::msg::VehicleLocalPosition>> sub1_; /**< Ground truth local position subscriber */
  std::shared_ptr<message_filters::Subscriber<px4_msgs::msg::VehicleAttitude>> sub2_;      /**< Ground truth attitude subscriber */
  std::shared_ptr<message_filters::TimeSynchronizer<
      px4_msgs::msg::VehicleLocalPosition, px4_msgs::msg::VehicleAttitude>> sync_msg_;   /**< Time synchronizer */
};

/**
* @brief Main entry point for the ground truth odometry node.
*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundTruthOdometry>());
  rclcpp::shutdown();
  return 0;
}
