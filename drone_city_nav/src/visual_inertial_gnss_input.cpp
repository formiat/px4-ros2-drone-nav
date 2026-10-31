#include "visual_inertial_gnss_input.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>

namespace drone_city_nav {
namespace {

// The fix's degrees in metres about the origin: the WGS84 equatorial radius
// over a flat earth, true to a centimetre over the kilometres a mission
// flies (0.3 percent of curvature over 2 km is 6 m of height, which the
// altitude carries, and a 2e-5 scale across).
constexpr double kEarthRadiusM{6378137.0};

[[nodiscard]] Eigen::Vector3d nedFromLlh(const Eigen::Vector3d& llh,
                                         const Eigen::Vector3d& origin) noexcept {
  const double to_radians = std::numbers::pi / 180.0;
  const double north = (llh.x() - origin.x()) * to_radians * kEarthRadiusM;
  const double east = (llh.y() - origin.y()) * to_radians * kEarthRadiusM *
                      std::cos(origin.x() * to_radians);
  const double down = -(llh.z() - origin.z());
  return {north, east, down};
}

[[nodiscard]] double normalised(const Eigen::Vector3d& residual,
                                const Eigen::Vector3d& variance) noexcept {
  double sum = 0.0;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    sum += residual(axis) * residual(axis) / std::max(variance(axis), 1.0e-12);
  }
  return std::sqrt(sum / 3.0);
}

} // namespace

GnssInputConfig gnssInputConfig(rclcpp::Node& node) {
  return GnssInputConfig{
      .trust =
          GnssTrustConfig{
              .window_s = node.declare_parameter<double>("gnss_window_s", 5.0),
              .position_gate =
                  node.declare_parameter<double>("gnss_position_gate", 1.0e300),
              .velocity_gate =
                  node.declare_parameter<double>("gnss_velocity_gate", 1.0e300),
              .self_gate_mps =
                  node.declare_parameter<double>("gnss_self_gate_mps", 1.0e300),
              .jump_gate_m =
                  node.declare_parameter<double>("gnss_jump_gate_m", 1.0e300),
              .clock_step_gate_s =
                  node.declare_parameter<double>("gnss_clock_step_gate_s", 1.0e300),
          },
      .reference_fixes = static_cast<std::size_t>(
          node.declare_parameter<int>("gnss_reference_fixes", 30)),
      .sigma_floor_m = node.declare_parameter<double>("gnss_sigma_floor_m", 0.5),
  };
}

GnssFixSample gnssFixSample(const px4_msgs::msg::SensorGps& fix,
                            const std::int64_t stamp_ns) noexcept {
  return GnssFixSample{
      .stamp_ns = stamp_ns,
      .valid = fix.fix_type >= px4_msgs::msg::SensorGps::FIX_TYPE_3D,
      .latitude_deg = fix.latitude_deg,
      .longitude_deg = fix.longitude_deg,
      .altitude_m = fix.altitude_msl_m,
      .velocity_ned_mps = Eigen::Vector3d{fix.vel_n_m_s, fix.vel_e_m_s, fix.vel_d_m_s},
      .horizontal_sigma_m = fix.eph,
      .vertical_sigma_m = fix.epv,
      .speed_sigma_mps = fix.s_variance_m_s,
  };
}

std::string GnssInput::summary(const GnssInputDecision& decision,
                               const px4_msgs::msg::SensorGps& fix,
                               const std::uint64_t fused) {
  char line[512];
  std::snprintf(
      line, sizeof line,
      "GNSS_TRUST trusted=%s agreeing=%s agreeing_s=%.1f reason=%s residual_m=%.2f "
      "normalised=%.2f velocity_mps=%.2f velocity_normalised=%.2f self_mps=%.2f "
      "step_m=%.2f "
      "clock_s=%.3f fix_type=%u eph=%.2f fused=%llu",
      decision.verdict.trusted ? "true" : "false",
      decision.verdict.agreeing ? "true" : "false", decision.verdict.agreeing_s,
      std::string{decision.verdict.reason}.c_str(), decision.position_residual_m,
      decision.reading.position_residual_normalised, decision.velocity_residual_mps,
      decision.reading.velocity_residual_normalised,
      decision.reading.self_disagreement_mps, decision.reading.position_step_m,
      decision.reading.clock_step_s, static_cast<unsigned>(fix.fix_type),
      static_cast<double>(fix.eph), static_cast<unsigned long long>(fused));
  return line;
}

GnssInputDecision
GnssInput::fix(const GnssFixSample& sample, const bool estimator_initialized,
               const VisualInertialEstimate* const predicted) noexcept {
  GnssInputDecision decision;
  const Eigen::Vector3d llh{sample.latitude_deg, sample.longitude_deg,
                            sample.altitude_m};
  if (!reference_llh_.has_value()) {
    // The origin from the fixes on the pad, before the vehicle moves: the
    // estimator's own origin is where it initializes at rest.
    if (sample.valid && llh.allFinite()) {
      reference_sum_ += llh;
      ++reference_count_;
      if (reference_count_ >= config_.reference_fixes) {
        reference_llh_ = reference_sum_ / static_cast<double>(reference_count_);
      }
    }
    return decision;
  }
  decision.reference_ready = true;
  decision.reading.stamp_ns = sample.stamp_ns;
  decision.reading.valid =
      sample.valid && llh.allFinite() && sample.velocity_ned_mps.allFinite();
  const Eigen::Vector3d ned = decision.reading.valid ? nedFromLlh(llh, *reference_llh_)
                                                     : Eigen::Vector3d::Zero();
  const double horizontal_sigma =
      std::max(sample.horizontal_sigma_m, config_.sigma_floor_m);
  const double vertical_sigma =
      std::max(sample.vertical_sigma_m, config_.sigma_floor_m);
  decision.position_ned_m = ned;
  decision.sigma_m = {horizontal_sigma, horizontal_sigma, vertical_sigma};
  if (previous_.has_value() && previous_ned_m_.has_value() && decision.reading.valid) {
    const double dt =
        static_cast<double>(sample.stamp_ns - previous_->stamp_ns) * 1.0e-9;
    decision.reading.clock_step_s = dt;
    decision.reading.position_step_m = (ned - *previous_ned_m_).norm();
    if (dt > 1.0e-3) {
      decision.reading.self_disagreement_mps =
          (sample.velocity_ned_mps - (ned - *previous_ned_m_) / dt).norm();
    }
  } else {
    decision.reading.clock_step_s = 0.0;
  }
  if (decision.reading.valid && estimator_initialized && predicted != nullptr) {
    const Eigen::Vector3d position_residual = ned - predicted->position_ned_m;
    const Eigen::Vector3d fix_variance{horizontal_sigma * horizontal_sigma,
                                       horizontal_sigma * horizontal_sigma,
                                       vertical_sigma * vertical_sigma};
    decision.position_residual_m = position_residual.norm();
    decision.reading.position_residual_normalised =
        normalised(position_residual, predicted->position_variance_m2 + fix_variance);
    const Eigen::Vector3d velocity_residual =
        sample.velocity_ned_mps - predicted->velocity_ned_mps;
    const double speed_sigma = std::max(sample.speed_sigma_mps, 0.1);
    decision.velocity_residual_mps = velocity_residual.norm();
    decision.reading.velocity_residual_normalised = normalised(
        velocity_residual, predicted->velocity_variance_m2ps2 +
                               Eigen::Vector3d::Constant(speed_sigma * speed_sigma));
  } else if (decision.reading.valid) {
    // No prediction to disagree with: the fix is read for itself alone.
    decision.reading.position_residual_normalised = 0.0;
    decision.reading.velocity_residual_normalised = 0.0;
  }
  decision.verdict = trust_.read(decision.reading);
  decision.fuse =
      decision.verdict.trusted && estimator_initialized && predicted != nullptr;
  if (decision.reading.valid) {
    previous_ = sample;
    previous_ned_m_ = ned;
  } else {
    previous_.reset();
    previous_ned_m_.reset();
  }
  return decision;
}

} // namespace drone_city_nav
