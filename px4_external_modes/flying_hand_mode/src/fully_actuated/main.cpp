#include <flying_hand_mode/runtime/external_mode.hpp>

#include <flying_hand_mode/fully_actuated/flying_hand_fully_actuated_controller.hpp>

#include <px4_ros2/components/message_compatibility_check.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<rclcpp::Node>("flying_hand_fully_actuated");
    std::string px4_prefix =
      node->declare_parameter<std::string>("px4_topic_namespace_prefix", "");
    if (!px4_prefix.empty() && px4_prefix.back() != '/') {
      px4_prefix.push_back('/');
    }

    if (!px4_ros2::waitForFMU(
        *node, rclcpp::Duration(std::chrono::seconds(15)), px4_prefix))
    {
      throw std::runtime_error("Timed out waiting for PX4");
    }
    if (!px4_ros2::messageCompatibilityCheck(
        *node,
        {{"fmu/in/vehicle_thrust_setpoint"},
          {"fmu/in/vehicle_torque_setpoint"},
          {"fmu/out/control_allocator_status"},
          {"fmu/out/vehicle_odometry"},
          {"fmu/out/sensor_combined"}},
        px4_prefix))
    {
      throw std::runtime_error("PX4 fully actuated wrench message compatibility check failed");
    }

    auto controller = std::make_shared<
      flying_hand_mode::fully_actuated::FlyingHandFullyActuatedController>(*node);
    flying_hand_mode::runtime::ControllerCallbacks callbacks;
    callbacks.update = [controller](const flying_hand_mode::runtime::ControllerInput & input) {
        return controller->update(input);
      };
    callbacks.accept = [controller]() {return controller->acceptPendingUpdate();};
    callbacks.reject = [controller]() {controller->rejectPendingUpdate();};
    callbacks.recover = [controller]() {controller->recoverAfterRejectedUpdate();};
    callbacks.reset = [controller]() {controller->reset();};

    flying_hand_mode::runtime::FlyingHandModeConfiguration mode_configuration;
    mode_configuration.mode_name = "Flying Hand Fully Actuated";
    mode_configuration.diagnostic_name = "flying_hand_fully_actuated/controller";
    mode_configuration.hardware_id = "paper_tilted_hex_4dof";
    mode_configuration.topic_prefix = "/flying_hand_fully_actuated";
    mode_configuration.require_calibration_confirmation = true;
    mode_configuration.monitor_control_allocator = true;
    auto mode = std::make_shared<flying_hand_mode::runtime::FlyingHandMode>(
      *node, std::move(mode_configuration),
      [controller](const flying_hand_mode::runtime::JointVector & joints) {
        return controller->endEffectorPoseFlu(joints);
      },
      px4_prefix, std::move(callbacks));
    auto executor =
      std::make_unique<flying_hand_mode::runtime::FlyingHandModeExecutor>(*mode, px4_prefix);
    if (!executor->doRegister()) {
      throw std::runtime_error("Flying Hand Fully Actuated mode registration failed");
    }

    const auto context = node->get_node_base_interface()->get_context();
    const std::weak_ptr<flying_hand_mode::runtime::FlyingHandMode> weak_mode = mode;
    const auto pre_shutdown_callback = context->add_pre_shutdown_callback(
      [weak_mode]() {
        if (const auto active_mode = weak_mode.lock()) {
          active_mode->prepareShutdown();
        }
      });

    RCLCPP_INFO(
      node->get_logger(),
      "Flying Hand fully actuated MPC is ready (%s)",
      node->get_parameter("closed_loop").as_bool() ? "closed loop" : "shadow");
    rclcpp::spin(node);
    (void)context->remove_pre_shutdown_callback(pre_shutdown_callback);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("flying_hand_fully_actuated"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
