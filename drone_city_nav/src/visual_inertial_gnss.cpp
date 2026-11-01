#include <cmath>

#include "visual_inertial_odometry_impl.hpp"

namespace drone_city_nav {

// The satellite fix as an absolute measurement of the position block, each
// axis weighted by its own standard deviation against the filter's pixel
// noise, as the held height is (holdHeight): the filter's own update does
// the rest, clones included.
void VisualInertialOdometry::addAbsolutePosition(const Eigen::Vector3d& position_ned_m,
                                                 const Eigen::Vector3d& sigma_m,
                                                 const double unaided_drift_m) {
  if (!impl_->initialized || !position_ned_m.allFinite() || !sigma_m.allFinite() ||
      !(sigma_m.minCoeff() > 0.0) || !std::isfinite(unaided_drift_m)) {
    return;
  }
  if (unaided_drift_m > 0.0) {
    impl_->covariance.block(kPosition, kPosition, 3, 3) +=
        unaided_drift_m * unaided_drift_m * Eigen::Matrix3d::Identity();
  }
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, impl_->covariance.rows());
  Eigen::VectorXd residual{3};
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    const double weight = impl_->config.observation_noise / sigma_m(axis);
    jacobian(axis, kPosition + axis) = weight;
    residual(axis) = weight * (position_ned_m(axis) - impl_->position(axis));
  }
  impl_->update(jacobian, residual);
}

} // namespace drone_city_nav
