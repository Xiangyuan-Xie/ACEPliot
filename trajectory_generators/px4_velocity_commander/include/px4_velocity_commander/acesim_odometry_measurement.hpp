#pragma once

#include <nav_msgs/msg/odometry.hpp>

#include <array>
#include <string>

enum class MeasurementSource
{
  PoseStamped,
  OdometryPose,
};

MeasurementSource parseMeasurementSource(const std::string & value);

std::array<double, 3> acesimOdometryPositionToEnu(const nav_msgs::msg::Odometry & msg);
