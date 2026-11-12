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
    // The drift of an odometry is a translation of its whole window: the
    // position and every clone's, together, not the position alone — the
    // features seen from the clones held the window and undid a shift of the
    // position by the next frame (r2650: the trust flapped four times more).
    Eigen::MatrixXd shift = Eigen::MatrixXd::Zero(impl_->covariance.rows(), 3);
    shift.block(kPosition, 0, 3, 3) = Eigen::Matrix3d::Identity();
    for (std::size_t i = 0U; i < impl_->clones.size(); ++i) {
      shift.block(kImuStates + kCloneStates * static_cast<Eigen::Index>(i) + kPosition,
                  0, 3, 3) = Eigen::Matrix3d::Identity();
    }
    impl_->covariance += unaided_drift_m * unaided_drift_m * shift * shift.transpose();
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

void VisualInertialOdometry::addAbsoluteVelocity(
    const Eigen::Vector3d& velocity_ned_mps, const double sigma_mps) {
  if (!impl_->initialized || !velocity_ned_mps.allFinite() ||
      !std::isfinite(sigma_mps) || !(sigma_mps > 0.0)) {
    return;
  }
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(3, impl_->covariance.rows());
  Eigen::VectorXd residual{3};
  const double weight = impl_->config.observation_noise / sigma_mps;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    jacobian(axis, kVelocity + axis) = weight;
    residual(axis) = weight * (velocity_ned_mps(axis) - impl_->velocity(axis));
  }
  impl_->update(jacobian, residual);
}

} // namespace drone_city_nav
