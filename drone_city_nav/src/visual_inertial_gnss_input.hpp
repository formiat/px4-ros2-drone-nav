#pragma once

// The satellite signal as the visual-inertial estimator's input (roadmap item
// 39, stages 1 and 2): the fix's frame aligned with the estimator's on the
// pad, every fix read against the estimator's prediction for the trust
// measurement (gnss_trust.hpp), and the fix handed to the filter only while
// the signal is trusted. ROS-free: the node feeds it plain numbers.

#include "drone_city_nav/autopilot_state.hpp"
#include "drone_city_nav/gnss_trust.hpp"
#include "drone_city_nav/visual_inertial_odometry.hpp"

#include <rclcpp/node.hpp>

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace drone_city_nav {

struct GnssFixSample {
  std::int64_t stamp_ns{0};
  bool valid{false}; // a 3D fix
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  Eigen::Vector3d velocity_ned_mps{Eigen::Vector3d::Zero()};
  double horizontal_sigma_m{0.0}; // the receiver's own figure (eph)
  double vertical_sigma_m{0.0};   // epv
  double speed_sigma_mps{0.0};    // s_variance
};

struct GnssInputConfig {
  GnssTrustConfig trust;
  std::size_t reference_fixes{30}; // fixes averaged on the pad for the frame's origin
  double sigma_floor_m{0.5};       // no fix is believed closer than this
};

struct GnssInputDecision {
  bool reference_ready{false};
  bool fuse{false}; // the fix goes to the filter
  Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d sigma_m{Eigen::Vector3d::Zero()};
  GnssFixReading reading;
  GnssTrustVerdict verdict;
  double position_residual_m{0.0};
  double velocity_residual_mps{0.0};
};

// The node's parameters: the gates' numbers are the measured flights'
// (urban_mvp.yaml, the open sky); 1e300 is no gate.
[[nodiscard]] GnssInputConfig gnssInputConfig(rclcpp::Node& node);

// The autopilot's fix (through its adapter) as the input's sample, at the stamp the
// node mapped.
[[nodiscard]] GnssFixSample gnssFixSample(const AutopilotGnssFix& fix,
                                          std::int64_t stamp_ns) noexcept;

class GnssInput {
public:
  explicit GnssInput(const GnssInputConfig& config) noexcept
      : config_{config},
        trust_{config.trust} {
  }

  // The fix against the estimator's prediction at the fix's moment; the
  // prediction is read only when the estimator is initialized.
  [[nodiscard]] GnssInputDecision fix(const GnssFixSample& sample,
                                      bool estimator_initialized,
                                      const VisualInertialEstimate* predicted) noexcept;

  [[nodiscard]] bool trusted() const noexcept {
    return trust_.trusted();
  }

  // No fix within the clock gate of the last one is the signal gone: the
  // trust is read with no fix at `now_ns` and ends, once per outage (the
  // law is read per fix, and an outage brings none: r2637 flew 90 s of
  // "lost" with the trust still declared). The verdict when it changes.
  [[nodiscard]] std::optional<GnssTrustVerdict> expire(std::int64_t now_ns) noexcept;

  // The log line of a fix: the trust's state and every reading behind it.
  [[nodiscard]] static std::string summary(const GnssInputDecision& decision,
                                           const AutopilotGnssFix& fix,
                                           std::uint64_t fused);

private:
  GnssInputConfig config_;
  GnssTrust trust_;
  // The frame's origin: the mean of the first fixes on the pad.
  std::size_t reference_count_{0U};
  Eigen::Vector3d reference_sum_{Eigen::Vector3d::Zero()}; // lat, lon, alt
  std::optional<Eigen::Vector3d> reference_llh_;
  std::optional<GnssFixSample> previous_;
  std::optional<Eigen::Vector3d> previous_ned_m_;
  bool expired_{false};
};

} // namespace drone_city_nav
