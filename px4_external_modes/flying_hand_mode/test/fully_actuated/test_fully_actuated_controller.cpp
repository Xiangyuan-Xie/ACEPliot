#include <flying_hand_mode/fully_actuated/flying_hand_fully_actuated_controller.hpp>
#include <flying_hand_mode/fully_actuated/fully_actuated_core.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

namespace flying_hand_mode::fully_actuated
{
namespace
{

class RosContext
{
public:
  RosContext()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  ~RosContext()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

RosContext g_ros_context;

TEST(FullyActuatedController, ProducesAFeasibleSixDimensionalHoverCommand)
{
  auto node = std::make_shared<rclcpp::Node>("fully_actuated_controller_test");
  FlyingHandFullyActuatedController controller(*node);
  flying_hand_mode::runtime::ControllerInput input;
  input.sample_timestamp_us = 1000;
  input.dt_s = 0.01;

  Eigen::Isometry3d frd_from_flu = Eigen::Isometry3d::Identity();
  frd_from_flu.linear() = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  input.current_ee_pose_ned =
    frd_from_flu * controller.endEffectorPoseFlu(input.arm_position_rad);
  input.target_ee_pose_ned = input.current_ee_pose_ned;

  const flying_hand_mode::runtime::ControllerOutput output = controller.update(input);
  ASSERT_TRUE(output.feasible);
  EXPECT_TRUE(output.normalizedCommandValid());
  EXPECT_LT(output.normalized_thrust_frd.z(), 0.0F);
  EXPECT_GT(output.allocation_condition_number, 1.0);
  EXPECT_TRUE(controller.acceptPendingUpdate());
}

TEST(FullyActuatedController, RejectsAConventionalVerticalHexGeometry)
{
  std::vector<double> vertical_axes;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    vertical_axes.insert(vertical_axes.end(), {0.0, 0.0, -1.0});
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("rotor.axis_frd", vertical_axes)});
  auto node = std::make_shared<rclcpp::Node>(
    "vertical_hex_controller_test", options);
  EXPECT_THROW(FlyingHandFullyActuatedController controller(*node), std::invalid_argument);
}

TEST(FullyActuatedController, RejectsMalformedRotorArrayAtStartup)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("rotor.position_frd_m", std::vector<double>(17, 0.0))});
  auto node = std::make_shared<rclcpp::Node>("malformed_rotor_array_test", options);
  EXPECT_THROW(FlyingHandFullyActuatedController controller(*node), std::invalid_argument);
}

TEST(FullyActuatedController, RejectsInvalidThrustCurveAtStartup)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter(
        "rotor.thrust_curve_kappa", std::vector<double>{0.3, 0.3, 1.1, 0.3, 0.3, 0.3})});
  auto node = std::make_shared<rclcpp::Node>("invalid_thrust_curve_test", options);
  EXPECT_THROW(FlyingHandFullyActuatedController controller(*node), std::invalid_argument);
}

}  // namespace
}  // namespace flying_hand_mode::fully_actuated
