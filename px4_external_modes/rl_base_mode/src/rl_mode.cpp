#include "rl_mode.hpp"
#include <px4_ros2/components/mode.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>

RLModeBase::RLModeBase(
  rclcpp::Node & node, std::unique_ptr<InferBackend> backend,
  px4_ros2::ModeBase::Settings settings, const std::string & topic_namespace_prefix,
  const std::string & root_dir)
: ModeBase(node, settings, topic_namespace_prefix), backend_(std::move(backend)),
  rnn_state_(node.get_logger())
{
  // Initialize logging parameters and model runtime resources.
  node.declare_parameter("flight_log_compression", true);

  FlightLogger::RosbagParams logger_params;
  logger_params.root_dir = root_dir;
  logger_params.enable_compression = node.get_parameter("flight_log_compression").as_bool();
  // logger_params.base_topic = "/" + settings.name;
  logger_params.base_topic = "";

  try {
    flight_logger_ = std::make_unique<FlightLogger>(logger_params);
    RCLCPP_INFO(
      node.get_logger(), "FlightLogger initialized successfully: %s",
      logger_params.root_dir.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node.get_logger(), "FlightLogger initialization failed: %s", e.what());
    flight_logger_ = nullptr;
  }

  // Use the base robot-data provider by default and initialize RNN hidden-state manager.
  robot_data_ = std::make_unique<RobotData>(*this);
  rnn_state_.initialize(backend_.get());
}

void RLModeBase::updateSetpoint(float dt_s)
{
  // 1. Update the robot's state from the latest sensor data
  robot_data_->updateState();

  // 2. Log odometry data
  logOdometryData();

  // 3. Get the observation tensor from the current state
  TensorMap inputs;
  getObservation(inputs, dt_s);

  // If observation is empty, it might mean the state is not yet valid.
  if (inputs.empty()) {
    return;
  }

  // 4. Log observations if logger is available
  logObservations(inputs, dt_s);

  // 5. Perform a forward pass through the network to get an action
  const auto inference_start = std::chrono::steady_clock::now();
  TensorMap action = backend_->forward(inputs, dt_s);
  const auto inference_end = std::chrono::steady_clock::now();
  const double inference_time_ms =
    std::chrono::duration<double, std::milli>(inference_end - inference_start).count();
  // Log inference latency to help diagnose real-time bottlenecks.
  if (flight_logger_ && flight_logger_->isOpen()) {
    std_msgs::msg::Float32 inference_msg;
    inference_msg.data = static_cast<float>(inference_time_ms);
    const double now_s = node().get_clock()->now().seconds();
    flight_logger_->log("inference_time_ms", inference_msg, now_s);
  }

  // 6. Log actions if logger is available
  logActions(action, dt_s);

  // 7. Apply the calculated action to the actuators
  applyAction(action, dt_s);
}

RobotData * RLModeBase::robotData()
{
  return robot_data_.get();
}

void RLModeBase::setRobotData(std::unique_ptr<RobotData> robot_data)
{
  robot_data_ = std::move(robot_data);
}

FlightLogger * RLModeBase::flightLogger()
{
  return flight_logger_.get();
}

RnnStateManager & RLModeBase::rnnState()
{
  return rnn_state_;
}

const RnnStateManager & RLModeBase::rnnState() const
{
  return rnn_state_;
}

InferBackend * RLModeBase::backend()
{
  return backend_.get();
}

const InferBackend * RLModeBase::backend() const
{
  return backend_.get();
}

void RLModeBase::logOdometryData()
{
  // Fast return when logger or robot data source is unavailable.
  if (!flight_logger_ || !flight_logger_->isOpen() || !robot_data_) {
    return;
  }

  // Read clock once to avoid repeated time-source calls.
  const auto now = node().get_clock()->now();
  const double now_s = now.seconds();
  const rclcpp::Time stamp = now;

  // Log pose as PoseStamped
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = "world";

  const auto & pos = robot_data_->RootPosW();
  pose_msg.pose.position.x = pos.x();
  pose_msg.pose.position.y = pos.y();
  pose_msg.pose.position.z = pos.z();

  const auto & quat = robot_data_->RootQuatW();
  pose_msg.pose.orientation.w = quat.w();
  pose_msg.pose.orientation.x = quat.x();
  pose_msg.pose.orientation.y = quat.y();
  pose_msg.pose.orientation.z = quat.z();

  flight_logger_->log("pose", pose_msg, now_s);

  // Log twist as TwistStamped
  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = stamp;
  twist_msg.header.frame_id = "base_link";

  const auto & lin_vel = robot_data_->RootLinVelB();
  twist_msg.twist.linear.x = lin_vel.x();
  twist_msg.twist.linear.y = lin_vel.y();
  twist_msg.twist.linear.z = lin_vel.z();

  const auto & ang_vel = robot_data_->RootAngVelB();
  twist_msg.twist.angular.x = ang_vel.x();
  twist_msg.twist.angular.y = ang_vel.y();
  twist_msg.twist.angular.z = ang_vel.z();

  flight_logger_->log("twist", twist_msg, now_s);

  // Log TF transform (world -> base_link)
  tf2_msgs::msg::TFMessage tf_msg;
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "world";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = pos.x();
  transform.transform.translation.y = pos.y();
  transform.transform.translation.z = pos.z();
  transform.transform.rotation.w = quat.w();
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  tf_msg.transforms.emplace_back(transform);
  flight_logger_->log("tf", tf_msg, now_s);
}

void RLModeBase::logTensor(const std::string & topic, const TensorMap & tensor_map, double now_s)
{
  // Serialize tensors by type into ROS messages and write to rosbag.
  for (const auto & [key, tensor] : tensor_map) {
    if (std::holds_alternative<std::vector<float>>(tensor)) {
      const auto & vec = std::get<std::vector<float>>(tensor);
      std_msgs::msg::Float32MultiArray msg;
      msg.data.insert(msg.data.end(), vec.begin(), vec.end());
      flight_logger_->log(topic + "/" + key, msg, now_s);
    } else if (std::holds_alternative<cv::Mat>(tensor)) {
      const auto & mat = std::get<cv::Mat>(tensor);
      sensor_msgs::msg::Image img_msg;
      img_msg.header.stamp = node().get_clock()->now();
      img_msg.header.frame_id = "camera_frame";
      try {
        auto bridge = cv_bridge::CvImage(img_msg.header, sensor_msgs::image_encodings::BGR8, mat);
        img_msg = *bridge.toImageMsg();
        flight_logger_->log(topic + "/" + key, img_msg, now_s);
      } catch (const cv_bridge::Exception & e) {
        try {
          auto bridge =
            cv_bridge::CvImage(img_msg.header, sensor_msgs::image_encodings::MONO8, mat);
          img_msg = *bridge.toImageMsg();
          flight_logger_->log(topic + "/" + key, img_msg, now_s);
        } catch (const cv_bridge::Exception & e2) {
          RCLCPP_ERROR(
            node().get_logger(), "Failed to convert cv::Mat to image for %s",
            key.c_str());
        }
      }
    }
  }
}

void RLModeBase::logObservations(const TensorMap & observations, float dt_s)
{
  (void)dt_s;
  if (!flight_logger_ || !flight_logger_->isOpen()) {return;}
  const double now_s = node().get_clock()->now().seconds();
  logTensor("observations", observations, now_s);
}

void RLModeBase::logActions(const TensorMap & actions, float dt_s)
{
  (void)dt_s;
  if (!flight_logger_ || !flight_logger_->isOpen()) {return;}
  const double now_s = node().get_clock()->now().seconds();
  logTensor("actions", actions, now_s);
}
