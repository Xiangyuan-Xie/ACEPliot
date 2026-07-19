#include <px4_velocity_commander/acesim_odometry_measurement.hpp>

#include <stdexcept>

MeasurementSource parseMeasurementSource(const std::string & value)
{
  if (value == "pose_stamped") {
    return MeasurementSource::PoseStamped;
  }
  if (value == "odometry_pose") {
    return MeasurementSource::OdometryPose;
  }
  throw std::invalid_argument(
          "measurement_source must be one of: pose_stamped, odometry_pose");
}

std::array<double, 3> acesimOdometryPositionToEnu(const nav_msgs::msg::Odometry & msg)
{
  const auto & position_nwu = msg.pose.pose.position;
  return {
    -position_nwu.y,
    position_nwu.x,
    position_nwu.z,
  };
}
