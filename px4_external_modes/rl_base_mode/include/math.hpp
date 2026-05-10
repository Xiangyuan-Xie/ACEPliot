#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>
#include <algorithm>
#include <array>
#include <cmath>

/**
 * @brief Computes the relative transform from frame 1 to frame 2.
 *
 * Given translation/rotation from frame 0 to frame 1, and optionally frame 0 to frame 2,
 * this function returns the transform from frame 1 to frame 2. If frame 0 to frame 2 is
 * not provided, frame 2 is assumed to coincide with frame 0.
 *
 * @param t01 Translation vector from frame 0 to frame 1.
 * @param q01 Rotation quaternion from frame 0 to frame 1.
 * @param t02 Optional translation vector from frame 0 to frame 2.
 * @param q02 Optional rotation quaternion from frame 0 to frame 2.
 * @return Pair {t12, q12}, the translation and rotation from frame 1 to frame 2.
 */
std::pair<Eigen::Vector3f, Eigen::Quaternionf>
subtract_frame_transforms(
  const Eigen::Vector3f & t01,
  const Eigen::Quaternionf & q01,
  const std::optional<Eigen::Vector3f> & t02 = std::nullopt,
  const std::optional<Eigen::Quaternionf> & q02 = std::nullopt);

template<typename T>
/**
 * @brief Clamps each vector element to the specified range.
 * @param vec Input vector.
 * @param min_val Lower bound.
 * @param max_val Upper bound.
 * @return New clamped vector.
 */
std::vector<T> clamp_vector(const std::vector<T> & vec, T min_val, T max_val)
{
  std::vector<T> result;
  result.reserve(vec.size());
  std::transform(
    vec.begin(), vec.end(), std::back_inserter(result),
    [min_val, max_val](T val) {return std::clamp(val, min_val, max_val);});
  return result;
}

/**
 * @brief Extracts intrinsic XYZ Euler angles (roll, pitch, yaw) from a quaternion.
 * @param q Input quaternion.
 * @return Tuple of (roll, pitch, yaw) in radians.
 */
std::tuple<float, float, float> euler_xyz_from_quat(const Eigen::Quaternionf & q);

/**
 * @brief Constructs a quaternion from intrinsic XYZ Euler angles.
 * @param roll Rotation about X axis in radians.
 * @param pitch Rotation about Y axis in radians.
 * @param yaw Rotation about Z axis in radians.
 * @return Resulting quaternion.
 */
Eigen::Quaternionf quat_from_euler_xyz(float roll, float pitch, float yaw);
/**
 * @brief Constructs a pure-yaw quaternion (roll=pitch=0).
 * @param yaw Rotation about Z axis in radians.
 * @return Resulting quaternion.
 */
Eigen::Quaternionf yawOnlyQuat(float yaw);


/**
 * @brief Converts a quaternion to a 3×3 rotation matrix.
 * @param q Input quaternion.
 * @return Corresponding rotation matrix.
 */
Eigen::Matrix3f rotation_matrix_from_quat(const Eigen::Quaternionf & q);

/**
 * @brief Wraps an angle in radians to [-pi, pi).
 * @param angle Input angle in radians.
 * @return Wrapped angle in radians.
 */
float wrapToPi(float angle);

/**
 * @brief Returns true when any axis command is active.
 * @tparam N Number of axes.
 * @param active Per-axis activation flags.
 * @return True if at least one element is true.
 */
template<std::size_t N>
bool anyAxisActive(const std::array<bool, N> & active)
{
  return std::any_of(active.begin(), active.end(), [](bool value) {return value;});
}
