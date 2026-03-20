#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <cstdint>

/**
* @brief Converts PX4 timestamp (in microseconds) to ROS2 header timestamp.
* @param timestamp PX4 timestamp in microseconds
* @return ROS2 standard message header with converted time
*/
std_msgs::msg::Header timestamp_to_header(uint64_t timestamp)
{
  std_msgs::msg::Header header;
  header.stamp.sec = static_cast<int32_t>(timestamp / 1'000'000);    // seconds
  header.stamp.nanosec = static_cast<uint32_t>((timestamp % 1'000'000) * 1000);    // remaining microseconds to ns
  return header;
}
