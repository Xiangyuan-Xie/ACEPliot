#include <math.hpp>

std::pair<Eigen::Vector3f, Eigen::Quaternionf>
subtract_frame_transforms(
  const Eigen::Vector3f & t01,
  const Eigen::Quaternionf & q01,
  const std::optional<Eigen::Vector3f> & t02,
  const std::optional<Eigen::Quaternionf> & q02)
{
  Eigen::Quaternionf q01n = q01;
  q01n.normalize();

  const Eigen::Quaternionf q10 = q01n.conjugate();
  Eigen::Quaternionf q12 = q02 ? (q10 * *q02) : q10;
  q12.normalize();

  const Eigen::Vector3f t2 = t02 ? *t02 : Eigen::Vector3f::Zero();
  const Eigen::Vector3f t12 = q10 * (t2 - t01);
  return {t12, q12};
}

std::tuple<float, float, float> euler_xyz_from_quat(const Eigen::Quaternionf & q)
{
  Eigen::Quaternionf qn = q.normalized();
  Eigen::Matrix3f R = qn.toRotationMatrix();
  // Intrinsic XYZ Euler angles (matches IsaacLab math_utils.euler_xyz_from_quat)
  float pitch = std::asin(-std::clamp(R(2, 0), -1.0f, 1.0f));
  float roll, yaw;
  if (std::abs(R(2, 0)) < 0.99999f) {
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    roll = std::atan2(-R(1, 2), R(1, 1));
    yaw = 0.0f;
  }
  return {roll, pitch, yaw};
}

Eigen::Quaternionf quat_from_euler_xyz(float roll, float pitch, float yaw)
{
  Eigen::Quaternionf qx(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()));
  Eigen::Quaternionf qy(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()));
  Eigen::Quaternionf qz(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
  return (qz * qy * qx).normalized();
}

Eigen::Matrix3f rotation_matrix_from_quat(const Eigen::Quaternionf & q)
{
  return q.normalized().toRotationMatrix();
}
