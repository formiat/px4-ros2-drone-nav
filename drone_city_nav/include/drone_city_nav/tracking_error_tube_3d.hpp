#pragma once

#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/tracking_error_tube_config_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace drone_city_nav {

// Exactly one occupancy source may be present. An unavailable source is
// deliberately neutral: missing distance evidence must not make unknown space
// less traversable than free space.
struct TrackingErrorTubeWorld3D {
  const ObservedOccupancyGrid3D* observed_occupancy{nullptr};
  const OccupancyGrid3D* occupancy{nullptr};
  // Fingerprint of occupied voxels only. Free/unknown relabeling deliberately
  // preserves this identity and therefore preserves the executable profile.
  std::uint64_t occupied_content_fingerprint{0U};
  const LaunchSupportContact3D* launch_support_contact{nullptr};
  const ProprioceptiveFreeSpaceSeed3D* proprioceptive_free_space_seed{nullptr};
};

struct TrackingErrorTubeProfile3D {
  bool valid{false};
  bool obstacle_evidence_available{false};
  TrackingErrorTubeConfig3D config{};
  SweptFootprintConfig physical_footprint{};
  std::vector<double> speed_limits_mps;
  double unconstrained_speed_limit_mps{0.0};
  double minimum_speed_limit_mps{0.0};
  double maximum_tracking_error_m{0.0};
  std::size_t constrained_segment_count{0U};
};

[[nodiscard]] bool
trackingErrorTubeProfile3DIsValid(const TrackingErrorTubeProfile3D& profile,
                                  std::size_t route_sample_count) noexcept;

// Diagnostics: the constrained station ranges of a profile, nearest first, as
// "begin_m-end_m:limit_mps@(x,y,z)" entries joined by ';', where the point is
// the sample holding the range's lowest limit. At most `maximum_ranges` are
// named, followed by "+N" for the ranges left out. Empty when nothing is
// constrained or the profile does not match the route.
[[nodiscard]] std::string
describeTrackingErrorTubeConstraints3D(std::span<const RouteSample3D> route,
                                       const TrackingErrorTubeProfile3D& profile,
                                       std::size_t maximum_ranges);

[[nodiscard]] bool
trackingErrorTubeProfile3DMatchesWorld(std::span<const RouteSample3D> route,
                                       const TrackingErrorTubeProfile3D& profile,
                                       const TrackingErrorTubeWorld3D& world);

enum class TrackingErrorTubeExecutionStatus3D : std::uint8_t {
  kAccepted,
  kInvalidProfile,
  kInvalidObservation,
  kSpeedLimitExceeded,
  kCrossTrackExceeded,
};

struct TrackingErrorTubeExecutionObservation3D {
  double station_m{0.0};
  double cross_track_error_m{0.0};
  double speed_mps{0.0};
};

struct TrackingErrorTubeExecutionAssessment3D {
  TrackingErrorTubeExecutionStatus3D status{
      TrackingErrorTubeExecutionStatus3D::kInvalidProfile};
  double speed_limit_mps{0.0};
  double tube_radius_m{0.0};

  [[nodiscard]] bool accepted() const noexcept {
    return status == TrackingErrorTubeExecutionStatus3D::kAccepted;
  }
};

[[nodiscard]] std::string_view trackingErrorTubeExecutionStatus3DName(
    TrackingErrorTubeExecutionStatus3D status) noexcept;

// Enforces the exact station-local speed/error contract sealed into a route.
// Point limits are shared by adjacent segments, so the lower endpoint limit is
// used throughout each segment and cannot overstate its validated clearance.
[[nodiscard]] TrackingErrorTubeExecutionAssessment3D assessTrackingErrorTubeExecution3D(
    std::span<const RouteSample3D> route, const TrackingErrorTubeProfile3D& profile,
    const TrackingErrorTubeExecutionObservation3D& observation) noexcept;

// Why a tube profile could not be built. A route the compiler refuses over its
// tube otherwise reaches the log as a bare invalid_tracking_error_tube, and a
// recorded flight held for three seconds on exactly that with nothing to act
// on.
enum class TrackingErrorTubeProfileStatus3D : std::uint8_t {
  kBuilt,
  kInvalidRoute,
  kInvalidWorld,
  kInvalidFootprint,
  kInvalidConfig,
  kInvalidSpeedCeiling,
  kNonFiniteSegmentLimit,
  kInvalidProfile,
};

[[nodiscard]] std::string_view
trackingErrorTubeProfileStatus3DName(TrackingErrorTubeProfileStatus3D status) noexcept;

// Produces per-station speed ceilings whose swept physical hull plus the
// speed-dependent tracking tube remains clear of confirmed occupied evidence.
// The route itself is never rejected merely because free/unknown labels differ.
// `status`, when given, names the rule a refusal was reached on.
[[nodiscard]] TrackingErrorTubeProfile3D makeTrackingErrorTubeProfile3D(
    std::span<const RouteSample3D> route, const TrackingErrorTubeWorld3D& world,
    const SweptFootprintConfig& physical_footprint,
    const TrackingErrorTubeConfig3D& config, double maximum_speed_mps,
    TrackingErrorTubeProfileStatus3D* status = nullptr);

} // namespace drone_city_nav
