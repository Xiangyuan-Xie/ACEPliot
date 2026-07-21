#include <robot_data.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>

namespace
{
const Eigen::Vector3f kGravityEnu{0.0f, 0.0f, -1.0f};
}  // namespace

RobotData::RobotData(px4_ros2::ModeBase & mode_base)
: mode_base_(&mode_base),
  root_pos_w_(0.f, 0.f, 0.f),
  root_quat_w_(1.f, 0.f, 0.f, 0.f),
  root_lin_vel_w_(0.f, 0.f, 0.f),
  root_ang_vel_w_(0.f, 0.f, 0.f),
  heading_w_(0.f),
  root_lin_vel_b_(0.f, 0.f, 0.f),
  root_ang_vel_b_(0.f, 0.f, 0.f),
  projected_gravity_b_(0.f, 0.f, -1.f)
{
  // Declare and read data-source configuration: PX4 native odometry or external ROS2 odometry.
  auto & node_ref = mode_base_->node();
  if (!node_ref.has_parameter("use_ros2_odom")) {
    node_ref.declare_parameter("use_ros2_odom", false);
  }
  if (!node_ref.has_parameter("odom_topic")) {
    node_ref.declare_parameter("odom_topic", "/odom");
  }

  use_ros2_odom_ = node_ref.get_parameter("use_ros2_odom").as_bool();
  odom_topic_ = node_ref.get_parameter("odom_topic").as_string();

  if (use_ros2_odom_) {
    // External odometry mode: subscribe directly and update state cache in callback.
    odom_subscription_ = node_ref.create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&RobotData::odomCallback, this, std::placeholders::_1));
  } else {
    // PX4 native mode: read state through px4_ros2 components.
    local_pos_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*mode_base_);
    attitude_ = std::make_shared<px4_ros2::OdometryAttitude>(*mode_base_);
    angular_vel_ = std::make_shared<px4_ros2::OdometryAngularVelocity>(*mode_base_);
  }
}

void RobotData::updateState()
{
  if (use_ros2_odom_) {
    return;
  }
  updatePX4State();
}

const Eigen::Vector3f & RobotData::RootPosW() const {return root_pos_w_;}
const Eigen::Quaternionf & RobotData::RootQuatW() const {return root_quat_w_;}
const Eigen::Vector3f & RobotData::RootLinVelW() const {return root_lin_vel_w_;}
const Eigen::Vector3f & RobotData::RootAngVelW() const {return root_ang_vel_w_;}
const Eigen::Vector3f & RobotData::RootLinVelB() const {return root_lin_vel_b_;}
const Eigen::Vector3f & RobotData::RootAngVelB() const {return root_ang_vel_b_;}
const Eigen::Vector3f & RobotData::ProjectedGravityB() const {return projected_gravity_b_;}
float RobotData::HeadingW() const {return heading_w_;}

void RobotData::updatePX4State()
{
  // Wait until all three PX4 odometry sources are valid before frame conversion.
  if (!local_pos_->lastValid() || !attitude_->lastValid() || !angular_vel_->lastValid()) {
    RCLCPP_WARN(mode_base_->node().get_logger(), "Waiting for PX4 odometry sources to be valid...");
    return;
  }

  const Eigen::Vector3f pos_ned = local_pos_->positionNed();
  const Eigen::Vector3f vel_ned = local_pos_->velocityNed();
  const float heading_ned = local_pos_->heading();
  const Eigen::Quaternionf q_ned = attitude_->attitude();
  const Eigen::Vector3f ang_vel_frd = angular_vel_->angularVelocityFrd();

  root_pos_w_ = px4_ros2::positionNedToEnu(pos_ned);
  root_lin_vel_w_ = px4_ros2::positionNedToEnu(vel_ned);
  root_quat_w_ = px4_ros2::attitudeNedToEnu(q_ned);
  heading_w_ = px4_ros2::yawNedToEnu(heading_ned);
  root_lin_vel_b_ = root_quat_w_.inverse() * root_lin_vel_w_;
  root_ang_vel_b_ = px4_ros2::frdToFlu(ang_vel_frd);
  root_ang_vel_w_ = root_quat_w_ * root_ang_vel_b_;
  projected_gravity_b_ = root_quat_w_.inverse() * kGravityEnu;
}

void RobotData::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Unpack position/attitude/velocity from external odometry and fill derived quantities.
  root_pos_w_ = Eigen::Vector3f(
    msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z);
  root_quat_w_ = Eigen::Quaternionf(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  root_lin_vel_b_ = Eigen::Vector3f(
    msg->twist.twist.linear.x, msg->twist.twist.linear.y,
    msg->twist.twist.linear.z);
  root_ang_vel_b_ = Eigen::Vector3f(
    msg->twist.twist.angular.x,
    msg->twist.twist.angular.y,
    msg->twist.twist.angular.z);
  root_lin_vel_w_ = root_quat_w_ * root_lin_vel_b_;
  root_ang_vel_w_ = root_quat_w_ * root_ang_vel_b_;
  projected_gravity_b_ = root_quat_w_.inverse() * kGravityEnu;

  const float w = root_quat_w_.w(), x = root_quat_w_.x(), y = root_quat_w_.y(),
    z = root_quat_w_.z();
  heading_w_ = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}
