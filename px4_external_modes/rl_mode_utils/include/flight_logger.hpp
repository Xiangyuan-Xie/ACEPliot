#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <variant>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

/**
 * @class FlightLogger
 * @brief Lightweight flight logger backed by rosbag2.
 */
class FlightLogger
{
public:
  /**
   * @struct RosbagParams
   * @brief Parameter bundle for rosbag writing behavior.
   */
  struct RosbagParams
  {
    std::string storage_id = "sqlite3";           ///< Preferred storage backend.
    bool enable_compression = true;               ///< Enables rosbag file compression.
    size_t max_bagfile_size = 512u * 1024u * 1024u;  ///< Maximum size per bag file.
    size_t max_bagfile_duration = 0;              ///< Maximum duration per bag file.
    std::string base_topic = "/flight_logger";    ///< Topic prefix used by the logger.
    std::string root_dir = "";                    ///< Root directory for output logs.
  };

  /// @brief Constructs a logger with default parameters.
  FlightLogger();

  /**
   * @brief Constructs a logger with custom rosbag parameters.
   * @param bagp Rosbag writing parameters.
   */
  explicit FlightLogger(const RosbagParams & bagp);

  /// @brief Destroys the logger and closes the writer if needed.
  ~FlightLogger();

  /// @brief Closes the rosbag writer explicitly.
  void close();

  /// @brief Returns whether the writer is currently open.
  bool isOpen() const;

  /**
   * @brief Writes a message to a sub-topic under the configured base topic.
   * @tparam T ROS message type.
   * @param topic Sub-topic name.
   * @param msg Message to write.
   * @param now_s Current time in seconds.
   */
  template<typename T>
  void log(const std::string & topic, const T & msg, double now_s);

private:
  /// @brief Builds a normalized full topic name from a sub-topic.
  std::string topic_(const std::string & sub_topic) const;

  template<typename T>
  struct dependent_false : std::false_type {};

  /// @brief Maps a ROS message type to its rosbag2 type string.
  template<typename T>
  static std::string type_name_of();

  /// @brief Normalizes a key into a valid topic segment.
  static std::string sanitize_key(std::string k);

  /// @brief Writes a message in a caller-owned locked context.
  template<typename T>
  void writeTopic_nolock(const std::string & name, const T & msg, uint64_t t_ns);

private:
  RosbagParams bag_p_;  ///< Logger configuration.
  std::mutex io_mx_;    ///< Mutex for thread-safe writes.
  std::unique_ptr<rosbag2_cpp::Writer> writer_;  ///< rosbag2 writer instance.
  std::set<std::string> created_topics_;  ///< Set of topics already created in the bag.
  std::unordered_map<std::string, std::size_t> registered_topics_types_;  ///< Topic-to-type hash cache.
};

#include <flight_logger.tpp>
