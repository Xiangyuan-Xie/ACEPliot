#include <flying_hand_mode/quadrotor/arm_kinematics.hpp>

#include <array>
#include <cmath>

namespace flying_hand_mode::quadrotor
{
namespace
{

struct JointOrigin
{
  Eigen::Vector3d xyz_m;
  Eigen::Vector3d rpy_rad;
};

struct LinkInertial
{
  double mass_kg;
  Eigen::Vector3d center_of_mass_m;
  Eigen::Matrix3d inertia_com_kg_m2;
};

Eigen::Matrix3d inertiaMatrix(
  double ixx, double ixy, double ixz, double iyy, double iyz, double izz) noexcept
{
  return (
    Eigen::Matrix3d() <<
      ixx, ixy, ixz,
      ixy, iyy, iyz,
      ixz, iyz, izz).finished();
}

const std::array<JointOrigin, kArmJointCount> kJointOrigins{{
  {{0.0, 0.0002, -0.1503}, {-1.5708, 0.0, 3.1416}},
  {{-0.04123, 0.2314, -0.0005}, {0.0, 0.0, 3.1416}},
  {{-0.04098, -0.2686, 0.0}, {0.0, 0.0, 0.0}},
  {{0.0, -0.05189, 0.0}, {1.5708, 1.5708, 0.0}},
}};

const Eigen::Vector3d kEndEffectorOffsetM{0.0, 0.008255, 0.0643};
const std::array<LinkInertial, kArmJointCount + 1> kLinkInertials{{
  {1.8403, {-0.013121, 0.00017397, -0.067418},
    inertiaMatrix(0.019333, 5.072e-05, 0.0010586, 0.02214, -4.352e-05, 0.028275)},
  {0.15156, {-0.025459, 0.17263, -0.00094115},
    inertiaMatrix(0.00087586, 0.0001761, -2.32e-06, 9.027e-05, 7.76e-06, 0.00092044)},
  {0.14766, {-0.039172, -0.18259, -0.00068453},
    inertiaMatrix(0.0016083, -4.92e-05, -1.8e-07, 4.02e-05, -8.08e-06, 0.0016046)},
  {0.0636, {0.0055284, -0.033186, -3.76e-06},
    inertiaMatrix(2.011e-05, 1.71e-06, 0.0, 1.802e-05, 0.0, 2.34e-05)},
  {0.15272, {-8.08936229013e-05, -0.0013581756244, 0.043125786869},
    inertiaMatrix(
      0.000114699427694, 1.57880455352e-06, 1.62506217408e-07,
      0.000340738958485, -4.73876972285e-06, 0.000267101289269)},
}};

Eigen::Matrix3d rotationFromRpy(const Eigen::Vector3d & rpy) noexcept
{
  return (
    Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

Eigen::Isometry3d originTransform(const JointOrigin & origin) noexcept
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = rotationFromRpy(origin.rpy_rad);
  transform.translation() = origin.xyz_m;
  return transform;
}

Eigen::Isometry3d jointRotation(double position_rad) noexcept
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() =
    Eigen::AngleAxisd(position_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return transform;
}

}  // namespace

bool VehicleMassProperties::allFinite() const noexcept
{
  return std::isfinite(mass_kg) && mass_kg > 0.0 && center_of_mass_flu_m.allFinite() &&
         inertia_com_flu_kg_m2.allFinite() && com_jacobian_flu_m_rad.allFinite();
}

Eigen::Isometry3d ArmKinematics::endEffectorPoseFlu(
  const JointVector & joint_position_rad) const noexcept
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (int joint = 0; joint < kArmJointCount; ++joint) {
    transform = transform * originTransform(kJointOrigins[static_cast<std::size_t>(joint)]) *
      jointRotation(joint_position_rad[joint]);
  }
  return transform * Eigen::Translation3d(kEndEffectorOffsetM);
}

Eigen::Matrix<double, 6, kArmJointCount> ArmKinematics::endEffectorJacobianFlu(
  const JointVector & joint_position_rad) const noexcept
{
  Eigen::Matrix<double, 6, kArmJointCount> jacobian;
  jacobian.setZero();
  std::array<Eigen::Vector3d, kArmJointCount> origins{};
  std::array<Eigen::Vector3d, kArmJointCount> axes{};

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (int joint = 0; joint < kArmJointCount; ++joint) {
    transform = transform * originTransform(kJointOrigins[static_cast<std::size_t>(joint)]);
    origins[static_cast<std::size_t>(joint)] = transform.translation();
    axes[static_cast<std::size_t>(joint)] = transform.linear() * Eigen::Vector3d::UnitZ();
    transform = transform * jointRotation(joint_position_rad[joint]);
  }
  const Eigen::Vector3d end_effector =
    (transform * Eigen::Translation3d(kEndEffectorOffsetM)).translation();

  for (int joint = 0; joint < kArmJointCount; ++joint) {
    const Eigen::Vector3d & axis = axes[static_cast<std::size_t>(joint)];
    jacobian.block<3, 1>(0, joint) =
      axis.cross(end_effector - origins[static_cast<std::size_t>(joint)]);
    jacobian.block<3, 1>(3, joint) = axis;
  }
  return jacobian;
}

VehicleMassProperties ArmKinematics::massPropertiesFlu(
  const JointVector & joint_position_rad) const noexcept
{
  VehicleMassProperties properties;
  if (!joint_position_rad.allFinite()) {
    return properties;
  }

  std::array<Eigen::Isometry3d, kArmJointCount + 1> link_transforms{};
  std::array<Eigen::Vector3d, kArmJointCount> joint_origins{};
  std::array<Eigen::Vector3d, kArmJointCount> joint_axes{};
  link_transforms[0] = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for (int joint = 0; joint < kArmJointCount; ++joint) {
    transform = transform * originTransform(kJointOrigins[static_cast<std::size_t>(joint)]);
    joint_origins[static_cast<std::size_t>(joint)] = transform.translation();
    joint_axes[static_cast<std::size_t>(joint)] =
      transform.linear() * Eigen::Vector3d::UnitZ();
    transform = transform * jointRotation(joint_position_rad[joint]);
    link_transforms[static_cast<std::size_t>(joint + 1)] = transform;
  }

  std::array<Eigen::Vector3d, kArmJointCount + 1> link_com_positions{};
  for (std::size_t link = 0; link < kLinkInertials.size(); ++link) {
    const LinkInertial & inertial = kLinkInertials[link];
    link_com_positions[link] = link_transforms[link] * inertial.center_of_mass_m;
    properties.mass_kg += inertial.mass_kg;
    properties.center_of_mass_flu_m += inertial.mass_kg * link_com_positions[link];
  }
  properties.center_of_mass_flu_m /= properties.mass_kg;

  for (std::size_t link = 0; link < kLinkInertials.size(); ++link) {
    const LinkInertial & inertial = kLinkInertials[link];
    const Eigen::Vector3d offset =
      link_com_positions[link] - properties.center_of_mass_flu_m;
    properties.inertia_com_flu_kg_m2 +=
      link_transforms[link].linear() * inertial.inertia_com_kg_m2 *
      link_transforms[link].linear().transpose() +
      inertial.mass_kg *
      (offset.squaredNorm() * Eigen::Matrix3d::Identity() - offset * offset.transpose());
  }

  for (int joint = 0; joint < kArmJointCount; ++joint) {
    for (std::size_t link = static_cast<std::size_t>(joint + 1);
      link < kLinkInertials.size(); ++link)
    {
      properties.com_jacobian_flu_m_rad.col(joint) +=
        kLinkInertials[link].mass_kg *
        joint_axes[static_cast<std::size_t>(joint)].cross(
        link_com_positions[link] - joint_origins[static_cast<std::size_t>(joint)]);
    }
    properties.com_jacobian_flu_m_rad.col(joint) /= properties.mass_kg;
  }
  return properties;
}

}  // namespace flying_hand_mode::quadrotor
