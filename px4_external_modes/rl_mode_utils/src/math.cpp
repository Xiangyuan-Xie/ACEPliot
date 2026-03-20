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
