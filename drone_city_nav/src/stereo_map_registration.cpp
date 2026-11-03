#include "stereo_map_registration.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <numbers>

namespace drone_city_nav {

StereoMapRegistration::StereoMapRegistration()
    : map_{PointPlaneMapConfig3D{.cell_m = kMapCellM,
                                 .maximum_points_per_cell = kMapPointsPerCell}} {
}

bool StereoMapRegistration::due(const std::int64_t stamp_ns) const noexcept {
  return !map_.empty() && stamp_ns - last_registration_ns_ >= kMapRegistrationPeriodNs;
}

MapFrameOffset StereoMapRegistration::target() const {
  const std::scoped_lock lock{mutex_};
  return target_;
}

MapFrameOffset StereoMapRegistration::applied() const {
  const std::scoped_lock lock{mutex_};
  return applied_;
}

StereoMapRegistrationReport StereoMapRegistration::registerFrame(
    const std::int64_t stamp_ns, const std::vector<Eigen::Vector3d>& points_body,
    const VisualInertialClonePose& pose, const bool act) {
  const auto started = std::chrono::steady_clock::now();
  last_registration_ns_ = stamp_ns;
  const MapFrameOffset frame = target();
  const Eigen::Vector3d prior = frame.apply(pose.position_ned_m);
  const Eigen::Matrix3d prior_rotation =
      frame.rotation() * pose.body_to_ned.toRotationMatrix();
  const PointPlaneRegistration3D registration = registerPointsToPlanes(
      map_, points_body, prior, Eigen::Quaterniond{prior_rotation},
      PointPlaneRegistrationConfig3D{.maximum_correspondence_m = 1.0,
                                     .maximum_iterations = 10U,
                                     .convergence_translation_m = 1.0e-3,
                                     .convergence_rotation_rad = 1.0e-4,
                                     .robust_width_m = 0.2,
                                     .born_by_ns = stamp_ns - kMapAgeNs});
  StereoMapRegistrationReport report;
  report.matched_fraction = registration.matched_fraction;
  report.matched_points =
      registration.matched_fraction * static_cast<double>(points_body.size());
  report.residual_rms_m = registration.residual_rms_m;
  report.information_per_point = registration.information_per_point;
  ++registrations_;
  const Eigen::Vector3d innovation = registration.position - prior;
  const Eigen::Matrix3d turn =
      registration.rotation.toRotationMatrix() * prior_rotation.transpose();
  report.innovation_m = innovation.norm();
  report.yaw_innovation_rad = std::atan2(turn(1, 0), turn(0, 0));
  Eigen::Vector3d correction = Eigen::Vector3d::Zero();
  double yaw_step = 0.0;
  if (registration.converged &&
      registration.matched_fraction >= kMapMinimumMatchedFraction &&
      report.matched_points >= static_cast<double>(kMapMinimumMatchedPoints) &&
      registration.residual_rms_m <= kMapMaximumResidualRmsM) {
    report.outcome = "far";
    if (innovation.norm() <= kMapCorrectionReachM) {
      recent_[recent_count_ % kMapConsistentReadings] = {innovation,
                                                         report.yaw_innovation_rad};
      ++recent_count_;
      if (recent_count_ >= kMapConsistentReadings) {
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        double mean_yaw = 0.0;
        for (const auto& [shift, yaw] : recent_) {
          mean += shift / static_cast<double>(kMapConsistentReadings);
          mean_yaw += yaw / static_cast<double>(kMapConsistentReadings);
        }
        report.consistent = true;
        for (const auto& [shift, yaw] : recent_) {
          report.consistent = report.consistent &&
                              (shift - mean).norm() <= kMapConsistentTranslationM &&
                              std::abs(yaw - mean_yaw) <= kMapConsistentYawRad;
        }
      }
      const Eigen::Matrix3d translational =
          registration.information.bottomRightCorner<3, 3>();
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver{translational};
      for (int axis = 0; axis < 3; ++axis) {
        if (solver.eigenvalues()(axis) / report.matched_points <
            kMapMinimumInformationPerPoint) {
          continue;
        }
        const Eigen::Vector3d direction = solver.eigenvectors().col(axis);
        correction += kMapCorrectionGain * direction.dot(innovation) * direction;
        ++report.observed_axes;
      }
      // The heading's information with the translation left free.
      const Eigen::Matrix<double, 1, 3> coupling =
          registration.information.block<1, 3>(2, 3);
      const double yaw_information =
          registration.information(2, 2) -
          coupling.dot(translational.ldlt().solve(coupling.transpose()));
      report.yaw_information_per_point = yaw_information / report.matched_points;
      if (report.yaw_information_per_point >= kMapMinimumYawInformationPerPoint &&
          std::abs(report.yaw_innovation_rad) <= kMapYawReachRad) {
        yaw_step = std::clamp(kMapYawGain * report.yaw_innovation_rad, -kMapYawStepRad,
                              kMapYawStepRad);
      }
      report.outcome =
          report.observed_axes > 0 || yaw_step != 0.0 ? "applied" : "degenerate";
      if (!report.consistent && std::strcmp(report.outcome, "applied") == 0) {
        report.outcome = "unsettled";
        correction.setZero();
        yaw_step = 0.0;
      }
    }
  }
  if (correction.norm() > kMapCorrectionStepM) {
    correction *= kMapCorrectionStepM / correction.norm();
  }
  report.correction_m = correction.norm();
  report.yaw_step_rad = yaw_step;
  {
    // The turn is about the vehicle, not about the frame's origin: the
    // frame's position there moves by the correction alone.
    const std::scoped_lock lock{mutex_};
    if (act) {
      const Eigen::Vector3d mapped = target_.apply(pose.position_ned_m) + correction;
      target_.yaw_rad += yaw_step;
      target_.translation = mapped - target_.rotation() * pose.position_ned_m;
    }
    report.target = target_;
  }
  if (report.observed_axes > 0 || yaw_step != 0.0) {
    ++corrections_;
  }
  report.registration_ms = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - started)
                               .count();
  report.scan_points = points_body.size();
  report.map_points = map_.pointCount();
  report.registrations = registrations_;
  report.corrections = corrections_;
  return report;
}

void StereoMapRegistration::lay(const std::int64_t stamp_ns,
                                std::vector<Eigen::Vector3d> points_body,
                                const VisualInertialClonePose& pose) {
  const MapFrameOffset frame = target();
  const Eigen::Matrix3d rotation =
      frame.rotation() * pose.body_to_ned.toRotationMatrix();
  const Eigen::Vector3d position = frame.apply(pose.position_ned_m);
  for (Eigen::Vector3d& point : points_body) {
    point = rotation * point + position;
  }
  map_.insert(points_body, 0U, stamp_ns);
}

void StereoMapRegistration::applyFrame(VisualInertialEstimate& estimate,
                                       const std::int64_t stamp_ns) {
  const std::scoped_lock lock{mutex_};
  const Eigen::Vector3d position = estimate.position_ned_m;
  if (applied_stamp_ns_ > 0 && stamp_ns > applied_stamp_ns_) {
    const double interval_s =
        1.0e-9 * static_cast<double>(stamp_ns - applied_stamp_ns_);
    const double yaw_reach = kMapYawRateRadps * interval_s;
    const double yaw_change =
        std::clamp(target_.yaw_rad - applied_.yaw_rad, -yaw_reach, yaw_reach);
    const Eigen::Vector3d published = applied_.apply(position);
    Eigen::Vector3d remaining = target_.apply(position) - published;
    const double reach = kMapOffsetRateMps * interval_s;
    if (remaining.norm() > reach) {
      remaining *= reach / remaining.norm();
    }
    applied_.yaw_rad += yaw_change;
    applied_.translation = published + remaining - applied_.rotation() * position;
  }
  applied_stamp_ns_ = std::max(applied_stamp_ns_, stamp_ns);
  const Eigen::Matrix3d rotation = applied_.rotation();
  estimate.position_ned_m = applied_.apply(position);
  estimate.velocity_ned_mps = rotation * estimate.velocity_ned_mps;
  estimate.body_to_ned =
      Eigen::Quaterniond{rotation * estimate.body_to_ned.toRotationMatrix()}
          .normalized();
}

std::string StereoMapRegistration::summary(const StereoMapRegistrationReport& report,
                                           const std::uint64_t frames_without_clone) {
  char line[640];
  std::snprintf(
      line, sizeof line,
      "VIO_MAP_REGISTRATION outcome=%s matched=%.2f matched_points=%.0f "
      "residual_m=%.3f information_per_point=%.4f observed_axes=%d "
      "innovation_m=%.3f correction_m=%.3f yaw_information_per_point=%.2f "
      "yaw_innovation_deg=%.2f yaw_step_deg=%.3f target_yaw_deg=%.2f "
      "target_translation=(%.2f,%.2f,%.2f) registration_ms=%.1f scan_points=%zu "
      "map_points=%zu registrations=%" PRIu64 " corrections=%" PRIu64
      " frames_without_clone=%" PRIu64,
      report.outcome, report.matched_fraction, report.matched_points,
      report.residual_rms_m, report.information_per_point, report.observed_axes,
      report.innovation_m, report.correction_m, report.yaw_information_per_point,
      report.yaw_innovation_rad * 180.0 / std::numbers::pi,
      report.yaw_step_rad * 180.0 / std::numbers::pi,
      report.target.yaw_rad * 180.0 / std::numbers::pi, report.target.translation.x(),
      report.target.translation.y(), report.target.translation.z(),
      report.registration_ms, report.scan_points, report.map_points,
      report.registrations, report.corrections, frames_without_clone);
  return line;
}

} // namespace drone_city_nav
