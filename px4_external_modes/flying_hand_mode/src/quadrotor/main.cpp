#include <flying_hand_mode/runtime/external_mode.hpp>

#include <flying_hand_mode/quadrotor/arm_kinematics.hpp>
#include <flying_hand_mode/quadrotor/flying_hand_controller.hpp>

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
    auto node = std::make_shared<rclcpp::Node>("flying_hand_quadrotor");
    std::string px4_prefix =
      node->declare_parameter<std::string>("px4_topic_namespace_prefix", "");
    if (!px4_prefix.empty() && px4_prefix.back() != '/') {
      px4_prefix.push_back('/');
    }

    if (!px4_ros2::waitForFMU(*node, rclcpp::Duration(std::chrono::seconds(15)), px4_prefix)) {
      throw std::runtime_error("Timed out waiting for PX4");
    }
    if (!px4_ros2::messageCompatibilityCheck(
        *node,
        {{"fmu/in/vehicle_thrust_setpoint"},
          {"fmu/in/vehicle_torque_setpoint"},
          {"fmu/out/vehicle_odometry"},
          {"fmu/out/sensor_combined"}},
        px4_prefix))
    {
      throw std::runtime_error("PX4 thrust/torque message compatibility check failed");
    }

    auto controller = std::make_shared<flying_hand_mode::quadrotor::FlyingHandController>(*node);
    flying_hand_mode::runtime::ControllerCallbacks controller_callbacks;
    controller_callbacks.update = [controller](
      const flying_hand_mode::runtime::ControllerInput & input) {
        return controller->update(input);
      };
    controller_callbacks.accept = [controller]() {return controller->acceptPendingUpdate();};
    controller_callbacks.reject = [controller]() {controller->rejectPendingUpdate();};
    controller_callbacks.recover =
      [controller]() {controller->recoverAfterRejectedUpdate();};
    controller_callbacks.reset = [controller]() {controller->reset();};
    const auto arm_kinematics =
      std::make_shared<flying_hand_mode::quadrotor::ArmKinematics>();
    flying_hand_mode::runtime::FlyingHandModeConfiguration mode_configuration;
    mode_configuration.mode_name = "Flying Hand Quadrotor";
    mode_configuration.diagnostic_name = "flying_hand_quadrotor/controller";
    mode_configuration.hardware_id = "x500_arm2x";
    mode_configuration.topic_prefix = "/flying_hand_quadrotor";
    auto mode = std::make_shared<flying_hand_mode::runtime::FlyingHandMode>(
      *node, std::move(mode_configuration),
      [arm_kinematics](const flying_hand_mode::runtime::JointVector & joints) {
        return arm_kinematics->endEffectorPoseFlu(joints);
      },
      px4_prefix, std::move(controller_callbacks));
    auto executor =
      std::make_unique<flying_hand_mode::runtime::FlyingHandModeExecutor>(*mode, px4_prefix);
    if (!executor->doRegister()) {
      throw std::runtime_error("Flying Hand External Mode registration failed");
    }

    const auto context = node->get_node_base_interface()->get_context();
    const std::weak_ptr<flying_hand_mode::runtime::FlyingHandMode> weak_mode = mode;
    const auto pre_shutdown_callback = context->add_pre_shutdown_callback(
      [weak_mode]() {
        if (const auto active_mode = weak_mode.lock()) {
          active_mode->prepareShutdown();
        }
      });

    RCLCPP_INFO(node->get_logger(), "Flying Hand whole-body controller is ready");
    rclcpp::spin(node);
    (void)context->remove_pre_shutdown_callback(pre_shutdown_callback);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("flying_hand_quadrotor"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
