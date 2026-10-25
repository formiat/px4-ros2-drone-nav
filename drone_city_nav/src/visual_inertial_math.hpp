#pragma once

#include <Eigen/Geometry>
#include <cmath>

namespace drone_city_nav {

// The rotation and gate arithmetic of the visual-inertial filter
// (visual_inertial_odometry.cpp), inline: a cross-product matrix, the
// exponential map of SO(3), a rotation re-orthonormalised through its
// quaternion, and the chi-square quantile of the gate (Wilson and Hilferty).
[[nodiscard]] inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

[[nodiscard]] inline Eigen::Matrix3d expSo3(const Eigen::Vector3d& rotation) {
  const double angle = rotation.norm();
  if (angle < 1.0e-9) {
    return Eigen::Matrix3d::Identity() + skew(rotation);
  }
  return Eigen::AngleAxisd{angle, rotation / angle}.toRotationMatrix();
}

[[nodiscard]] inline Eigen::Matrix3d orthonormalized(const Eigen::Matrix3d& rotation) {
  return Eigen::Quaterniond{rotation}.normalized().toRotationMatrix();
}

// The quantile of chi-square with `degrees` degrees of freedom at the normal
// quantile `z`.
[[nodiscard]] inline double chiSquareQuantile(const Eigen::Index degrees,
                                              const double z) {
  const double k = static_cast<double>(degrees);
  const double term = 1.0 - 2.0 / (9.0 * k) + z * std::sqrt(2.0 / (9.0 * k));
  return k * term * term * term;
}

} // namespace drone_city_nav
