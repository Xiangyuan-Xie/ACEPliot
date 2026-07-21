template<typename T>
void FlightLogger::log(const std::string & topic, const T & msg, double now_s)
{
  const std::string sanitized_sub_topic = sanitize_key(topic);
  const std::string full_topic_name = topic_(sanitized_sub_topic);
  constexpr auto kLoggerName = "flight_logger";

  if (!isOpen()) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "FlightLogger::log: Logger is closed. Skipping write operation for topic '%s'",
      full_topic_name.c_str()
    );
    return;
  }

  const uint64_t t_ns = static_cast<uint64_t>(now_s * 1e9);
  std::scoped_lock lk(io_mx_);

  if (!isOpen()) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "FlightLogger::log: Logger is closed. Skipping write operation for topic '%s'",
      full_topic_name.c_str()
    );
    return;
  }

  auto it = registered_topics_types_.find(full_topic_name);
  if (it != registered_topics_types_.end()) {
    if (it->second != typeid(T).hash_code()) {
      return;
    }
  } else {
    registered_topics_types_[full_topic_name] = typeid(T).hash_code();
  }

  writeTopic_nolock(full_topic_name, msg, t_ns);
}

template<typename T>
std::string FlightLogger::type_name_of()
{
  if constexpr (std::is_same_v<T, nav_msgs::msg::Odometry>) {
    return "nav_msgs/msg/Odometry";
  } else if constexpr (std::is_same_v<T, nav_msgs::msg::Path>) {
    return "nav_msgs/msg/Path";
  } else if constexpr (std::is_same_v<T, geometry_msgs::msg::Vector3>) {
    return "geometry_msgs/msg/Vector3";
  } else if constexpr (std::is_same_v<T, std_msgs::msg::Float32>) {
    return "std_msgs/msg/Float32";
  } else if constexpr (std::is_same_v<T, std_msgs::msg::Float64>) {
    return "std_msgs/msg/Float64";
  } else if constexpr (std::is_same_v<T, std_msgs::msg::Int32>) {
    return "std_msgs/msg/Int32";
  } else if constexpr (std::is_same_v<T, std_msgs::msg::String>) {
    return "std_msgs/msg/String";
  } else if constexpr (std::is_same_v<T, std_msgs::msg::Float32MultiArray>) {
    return "std_msgs/msg/Float32MultiArray";
  } else if constexpr (std::is_same_v<T, sensor_msgs::msg::Image>) {
    return "sensor_msgs/msg/Image";
  } else if constexpr (std::is_same_v<T, geometry_msgs::msg::PoseStamped>) {
    return "geometry_msgs/msg/PoseStamped";
  } else if constexpr (std::is_same_v<T, geometry_msgs::msg::TwistStamped>) {
    return "geometry_msgs/msg/TwistStamped";
  } else if constexpr (std::is_same_v<T, tf2_msgs::msg::TFMessage>) {
    return "tf2_msgs/msg/TFMessage";
  } else {
    static_assert(
      dependent_false<T>::value,
      "Unsupported ROS msg type. Please add the new type to type_name_of().");
  }
}

template<typename T>
void FlightLogger::writeTopic_nolock(const std::string & name, const T & msg, uint64_t t_ns)
{
  if (!writer_) {
    return;
  }

  try {
    if (created_topics_.find(name) == created_topics_.end()) {
      writer_->create_topic({name, type_name_of<T>(), "cdr", ""});
      created_topics_.insert(name);
    }
    rclcpp::SerializedMessage ser;
    rclcpp::Serialization<T> sz;
    sz.serialize_message(&msg, &ser);
    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_msg->topic_name = name;
    bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>(
      ser.get_rcl_serialized_message());
    bag_msg->time_stamp = t_ns;
    writer_->write(bag_msg);
  } catch (const std::exception & e) {
  }
}
