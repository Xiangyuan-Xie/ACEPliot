#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>

using namespace std::chrono_literals; // NOLINT

static const std::string topic_namespace_prefix = "";

/**
 * @class LocalNavigation
 * @brief A wrapper class for converting external pose data to PX4 local position measurements.
 *
 * This class supports multiple ROS message types and handles coordinate
 * frame conversions (e.g., ENU to NED) and application of extrinsic transformations.
 */
class LocalNavigation : public px4_ros2::LocalPositionMeasurementInterface
{
public:
  /**
   * @brief Constructor initializing the navigation interface and default extrinsics.
   * @param node Reference to the ROS node.
   */
  explicit LocalNavigation(rclcpp::Node & node)
  : LocalPositionMeasurementInterface(node, px4_ros2::PoseFrame::LocalNED,
      px4_ros2::VelocityFrame::LocalNED)
    // px4_ros2::VelocityFrame::LocalNED, topic_namespace_prefix)
  {
    // Initialize extrinsic parameters to identity
    extrinsic_translation_ = Eigen::Vector3d::Zero();
    extrinsic_rotation_ = Eigen::Quaterniond::Identity();
  }

  /**
   * @brief Process nav_msgs::msg::Odometry messages and update local position.
   * @param msg The received odometry message.
   */
  void updateLocalPosition(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    px4_ros2::LocalPositionMeasurement local_position_measurement {};

    local_position_measurement.timestamp_sample = msg->header.stamp;

    // Apply extrinsic transformation to position and orientation
    Eigen::Vector3d input_position(msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    Eigen::Quaterniond input_orientation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    Eigen::Vector3d transformed_position = extrinsic_rotation_ * input_position +
      extrinsic_translation_;
    Eigen::Quaterniond transformed_orientation = extrinsic_rotation_ * input_orientation;

    // PX4 expects NED frame, convert ENU to NED
    local_position_measurement.position_xy = Eigen::Vector2f(
      transformed_position.y(), transformed_position.x());
    local_position_measurement.position_xy_variance = Eigen::Vector2f(0.000000001f, 0.000000001f);
    // local_position_measurement.position_xy_variance = Eigen::Vector2f {msg->pose.covariance[7], msg->pose.covariance[0]};

    local_position_measurement.position_z = -transformed_position.z();
    local_position_measurement.position_z_variance = 0.000000001f;
    // local_position_measurement.position_z_variance = msg->pose.covariance[14];

    // Velocity transformation (if available)
    if (msg->twist.twist.linear.x != 0.0 || msg->twist.twist.linear.y != 0.0 ||
      msg->twist.twist.linear.z != 0.0)
    {
      Eigen::Vector3d input_velocity(msg->twist.twist.linear.x, msg->twist.twist.linear.y,
        msg->twist.twist.linear.z);
      Eigen::Vector3d transformed_velocity = extrinsic_rotation_ * input_velocity;

      local_position_measurement.velocity_xy = Eigen::Vector2f(
        transformed_velocity.x(), -transformed_velocity.y());
      local_position_measurement.velocity_xy_variance = Eigen::Vector2f(0.000000001f, 0.000000001f);
      // local_position_measurement.velocity_xy_variance = Eigen::Vector2f {msg->twist.covariance[7], msg->twist.covariance[0]};

      local_position_measurement.velocity_z = -transformed_velocity.z();
      local_position_measurement.velocity_z_variance = 0.000000001f;
      // local_position_measurement.velocity_z_variance = msg->twist.covariance[14];
    }

    // Orientation transformation and conversion
    Eigen::Quaternionf q_px4 = px4_ros2::attitudeEnuToNed(
      Eigen::Quaternionf(
        transformed_orientation.w(),
        transformed_orientation.x(),
        transformed_orientation.y(),
        transformed_orientation.z())
    );
    local_position_measurement.attitude_quaternion = q_px4;
    local_position_measurement.attitude_variance = Eigen::Vector3f(0.0001, 0.0001, 0.0001);
    // local_position_measurement.attitude_variance = Eigen::Vector3f {msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35]};

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

  /**
   * @brief Process geometry_msgs::msg::PoseStamped messages and update local position.
   * @param msg The received pose message.
   */
  void updateLocalPosition(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    px4_ros2::LocalPositionMeasurement local_position_measurement {};

    local_position_measurement.timestamp_sample = msg->header.stamp;

    Eigen::Vector3d input_position(msg->pose.position.x, msg->pose.position.y,
      msg->pose.position.z);
    Eigen::Quaterniond input_orientation(msg->pose.orientation.w, msg->pose.orientation.x,
      msg->pose.orientation.y, msg->pose.orientation.z);

    Eigen::Vector3d transformed_position = extrinsic_rotation_ * input_position +
      extrinsic_translation_;
    Eigen::Quaterniond transformed_orientation = extrinsic_rotation_ * input_orientation;

    local_position_measurement.position_xy = Eigen::Vector2f(
      transformed_position.y(), transformed_position.x());
    local_position_measurement.position_xy_variance = Eigen::Vector2f(0.000000001f, 0.000000001f);

    local_position_measurement.position_z = -transformed_position.z();
    local_position_measurement.position_z_variance = 0.000000001f;

    Eigen::Quaternionf q_px4 = px4_ros2::attitudeEnuToNed(
      Eigen::Quaternionf(
        transformed_orientation.w(),
        transformed_orientation.x(),
        transformed_orientation.y(),
        transformed_orientation.z())
    );
    local_position_measurement.attitude_quaternion = q_px4;
    local_position_measurement.attitude_variance = Eigen::Vector3f(0.0001, 0.0001, 0.0001);

    try {
      update(local_position_measurement);
      RCLCPP_DEBUG(
        _node.get_logger(),
        "Successfully sent pose update to navigation interface.");
    } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
      RCLCPP_ERROR_THROTTLE(
        _node.get_logger(),
        *_node.get_clock(), 1000, "Exception caught: %s", e.what());
    }
  }

  /**
   * @brief Process geometry_msgs::msg::PoseWithCovarianceStamped messages and update local position.
   * @param msg The received pose with covariance message.
   */
  void updateLocalPosition(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    px4_ros2::LocalPositionMeasurement local_position_measurement {};

    local_position_measurement.timestamp_sample = msg->header.stamp;

    Eigen::Vector3d input_position(msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    Eigen::Quaterniond input_orientation(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    Eigen::Vector3d transformed_position = extrinsic_rotation_ * input_position +
      extrinsic_translation_;
    Eigen::Quaterniond transformed_orientation = extrinsic_rotation_ * input_orientation;

    local_position_measurement.position_xy = Eigen::Vector2f(
      transformed_position.y(), transformed_position.x());
    local_position_measurement.position_xy_variance = Eigen::Vector2f(
      msg->pose.covariance[7],
      msg->pose.covariance[0]);

    local_position_measurement.position_z = -transformed_position.z();
    local_position_measurement.position_z_variance = msg->pose.covariance[14];

    Eigen::Quaternionf q_px4 = px4_ros2::attitudeEnuToNed(
      Eigen::Quaternionf(
        transformed_orientation.w(),
        transformed_orientation.x(),
        transformed_orientation.y(),
        transformed_orientation.z())
    );
    local_position_measurement.attitude_quaternion = q_px4;
    local_position_measurement.attitude_variance = Eigen::Vector3f(
      msg->pose.covariance[21],
      msg->pose.covariance[28],
      msg->pose.covariance[35]);

    try {
      update(local_position_measurement);
      RCLCPP_DEBUG(
        _node.get_logger(),
        "Successfully sent pose with covariance update to navigation interface.");
    } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
      RCLCPP_ERROR_THROTTLE(
        _node.get_logger(),
        *_node.get_clock(), 1000, "Exception caught: %s", e.what());
    }
  }

  /**
   * @brief Set extrinsic transformation from mocap frame to robot body frame.
   * @param translation Translation vector [x, y, z] from mocap origin to robot body origin.
   * @param rotation_matrix Row-major 3x3 rotation matrix as a 9-element vector.
   */
  void setExtrinsicParameters(
    const std::vector<double> & translation,
    const std::vector<double> & rotation_matrix)
  {
    if (translation.size() != 3) {
      RCLCPP_ERROR(_node.get_logger(), "Translation vector must have 3 elements [x, y, z]");
      return;
    }
    if (rotation_matrix.size() != 9) {
      RCLCPP_ERROR(_node.get_logger(), "Rotation matrix must have 9 elements [r11, ..., r33]");
      return;
    }

    extrinsic_translation_ = Eigen::Vector3d(translation[0], translation[1], translation[2]);

    Eigen::Matrix3d rotation_mat;
    rotation_mat << rotation_matrix[0], rotation_matrix[1], rotation_matrix[2],
      rotation_matrix[3], rotation_matrix[4], rotation_matrix[5],
      rotation_matrix[6], rotation_matrix[7], rotation_matrix[8];

    extrinsic_rotation_ = Eigen::Quaterniond(rotation_mat);
    extrinsic_rotation_.normalize();
  }

private:
  Eigen::Vector3d extrinsic_translation_;
  Eigen::Quaterniond extrinsic_rotation_;

};

/**
 * @class GenericOdometryNode
 * @brief ROS2 node for converting various pose/odometry messages into PX4 local position updates.
 *
 * This node subscribes to selected message types, applies extrinsic transformations,
 * and pushes position data to PX4 via the LocalNavigation interface.
 */
class GenericOdometryNode : public rclcpp::Node
{
public:
  GenericOdometryNode()
  : Node("generic_odometry_node")
  {
    _interface = std::make_unique<LocalNavigation>(*this);

    // Declare parameters
    this->declare_parameter("message_type", "odometry");
    this->declare_parameter("odometry_topic", "/Odometry");
    // Declare extrinsic parameters
    this->declare_parameter("extrinsic_translation", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter(
      "extrinsic_rotation_matrix", std::vector<double>{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    });

    // Read and set extrinsics
    auto translation = this->get_parameter("extrinsic_translation").as_double_array();
    auto rotation_matrix = this->get_parameter("extrinsic_rotation_matrix").as_double_array();
    _interface->setExtrinsicParameters(translation, rotation_matrix);

    if (!_interface->doRegister()) {
      throw std::runtime_error("Registration failed");
    }

    // Set up QoS
    const rclcpp::QoS qos(rclcpp::QoS(1)
      .best_effort()
      .keep_last(1)
      .durability_volatile());

    // Subscribe to appropriate topic based on message type
    std::string message_type = this->get_parameter("message_type").as_string();

    if (message_type == "odometry") {
      odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("odometry_topic").as_string(),
        qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          _interface->updateLocalPosition(msg);
        });
      RCLCPP_INFO(
        this->get_logger(), "Subscribed to odometry topic: %s",
        this->get_parameter("odometry_topic").as_string().c_str());
    } else if (message_type == "pose_stamped") {
      pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        this->get_parameter("odometry_topic").as_string(),
        qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          _interface->updateLocalPosition(msg);
        });
      RCLCPP_INFO(
        this->get_logger(), "Subscribed to pose topic: %s",
        this->get_parameter("odometry_topic").as_string().c_str());
    } else if (message_type == "pose_with_covariance_stamped") {
      pose_with_covariance_sub_ =
        this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        this->get_parameter("odometry_topic").as_string(),
        qos,
        [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
          _interface->updateLocalPosition(msg);
        });
      RCLCPP_INFO(
        this->get_logger(), "Subscribed to pose with covariance topic: %s",
        this->get_parameter("odometry_topic").as_string().c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Unsupported message type: %s", message_type.c_str());
      RCLCPP_INFO(
        this->get_logger(),
        "Supported message types: odometry, pose_stamped, pose_with_covariance_stamped");
    }
  }

private:
  std::unique_ptr<LocalNavigation> _interface;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_with_covariance_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GenericOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
