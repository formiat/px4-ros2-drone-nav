#include "drone_city_nav/compiled_trajectory_3d.hpp"

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>
#include <utility>

namespace drone_city_nav {
namespace {

class TrajectoryHasher final {
public:
  void value(const std::uint64_t input) noexcept {
    for (std::size_t byte = 0U; byte < sizeof(input); ++byte) {
      hash_ ^= (input >> (byte * 8U)) & 0xffU;
      hash_ *= kFnvPrime;
    }
  }

  void signedValue(const std::int64_t input) noexcept {
    value(static_cast<std::uint64_t>(input));
  }

  void boolean(const bool input) noexcept {
    value(input ? 1U : 0U);
  }

  void number(const double input) noexcept {
    if (!std::isfinite(input)) {
      valid_ = false;
      return;
    }
    value(input == 0.0 ? 0U : std::bit_cast<std::uint64_t>(input));
  }

  void text(const std::string_view input) noexcept {
    value(static_cast<std::uint64_t>(input.size()));
    for (const char character : input) {
      value(static_cast<std::uint8_t>(character));
    }
  }

  void point(const Point3& input) noexcept {
    number(input.x);
    number(input.y);
    number(input.z);
  }

  void vector(const Vec3& input) noexcept {
    number(input.x);
    number(input.y);
    number(input.z);
  }

  [[nodiscard]] std::uint64_t result() const noexcept {
    if (!valid_) {
      return 0U;
    }
    return hash_ == 0U ? 1U : hash_;
  }

private:
  static constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
  static constexpr std::uint64_t kFnvPrime{1099511628211ULL};

  std::uint64_t hash_{kFnvOffset};
  bool valid_{true};
};

void hashVehicleState(TrajectoryHasher& hash, const VehicleState3D& state) noexcept {
  hash.value(state.identity.revision);
  hash.value(state.identity.source_timestamp_us);
  hash.signedValue(state.identity.receive_stamp_ns);
  hash.point(state.position);
  hash.vector(state.velocity);
  hash.number(state.yaw_rad);
  hash.number(state.yaw_rate_radps);
}

void hashSegmentSpan(TrajectoryHasher& hash,
                     const PassageTraversalSegmentSpan& span) noexcept {
  hash.text(span.passage_segment_id.value());
  hash.number(span.begin_station_m);
  hash.number(span.end_station_m);
}

void hashConstrainedSpan(TrajectoryHasher& hash,
                         const ConstrainedRouteSpan& span) noexcept {
  hash.text(span.passage_traversal_id.value());
  hash.value(span.route_generation);
  hash.value(static_cast<std::uint64_t>(span.direction_sign));
  hash.number(span.begin_station_m);
  hash.number(span.end_station_m);
  hash.value(static_cast<std::uint64_t>(span.envelope.size()));
  for (const RouteEnvelopeSample& sample : span.envelope) {
    hash.number(sample.station_m);
    hash.number(sample.lateral_free_left_m);
    hash.number(sample.lateral_free_right_m);
    hash.number(sample.min_z_m);
    hash.number(sample.max_z_m);
    hash.number(sample.minimum_clearance_m);
    hash.number(sample.reference_z_m);
    hash.number(sample.reference_speed_mps);
  }
  hash.value(static_cast<std::uint64_t>(span.segment_spans.size()));
  for (const PassageTraversalSegmentSpan& segment : span.segment_spans) {
    hashSegmentSpan(hash, segment);
  }
}

void hashSweptFootprint(TrajectoryHasher& hash,
                        const SweptFootprintConfig& footprint) noexcept {
  hash.number(footprint.radius_m);
  hash.number(footprint.lower_extent_m);
  hash.number(footprint.upper_extent_m);
  hash.value(static_cast<std::uint64_t>(footprint.perimeter_samples));
  hash.value(static_cast<std::uint64_t>(footprint.radial_rings));
  hash.value(static_cast<std::uint64_t>(footprint.axial_samples));
  hash.number(footprint.sweep_step_m);
  hash.number(footprint.safe_clearance_threshold_m);
}

void hashRoute(TrajectoryHasher& hash,
               const std::vector<RouteSample3D>& route) noexcept {
  hash.value(static_cast<std::uint64_t>(route.size()));
  for (const RouteSample3D& sample : route) {
    hash.point(sample.position);
    hash.vector(sample.tangent);
    hash.number(sample.station_m);
    hash.number(sample.reference_speed_mps);
    hash.value(static_cast<std::uint64_t>(sample.required_risk_tier));
    hash.value(static_cast<std::uint64_t>(sample.transition));
  }
}

void hashTrackingErrorTube(TrajectoryHasher& hash,
                           const TrackingErrorTubeProfile3D& profile) noexcept {
  hash.boolean(profile.valid);
  hash.boolean(profile.obstacle_evidence_available);
  hash.number(profile.config.response_time_s);
  hashSweptFootprint(hash, profile.physical_footprint);
  hash.value(static_cast<std::uint64_t>(profile.speed_limits_mps.size()));
  for (const double speed_limit_mps : profile.speed_limits_mps) {
    hash.number(speed_limit_mps);
  }
  hash.number(profile.unconstrained_speed_limit_mps);
  hash.number(profile.minimum_speed_limit_mps);
  hash.number(profile.maximum_tracking_error_m);
  hash.value(static_cast<std::uint64_t>(profile.constrained_segment_count));
}

void hashTrajectoryResources(TrajectoryHasher& hash,
                             const CompiledTrajectory3D& trajectory) noexcept {
  hashRoute(hash, *trajectory.route);
  hash.value(static_cast<std::uint64_t>(trajectory.constrained_spans->size()));
  for (const ConstrainedRouteSpan& span : *trajectory.constrained_spans) {
    hashConstrainedSpan(hash, span);
  }
}

[[nodiscard]] std::uint64_t
calculateCompiledTrajectoryRevision(const CompiledTrajectory3D& trajectory) noexcept {
  if (!compiledTrajectoryResourcesVerified3D(trajectory)) {
    return 0U;
  }

  TrajectoryHasher hash;
  hashVehicleState(hash, trajectory.exact_initial_state);
  hash.value(static_cast<std::uint64_t>(trajectory.endpoint_semantics));
  hashTrajectoryResources(hash, trajectory);
  hashTrackingErrorTube(hash, *trajectory.tracking_error_tube);
  hash.number(trajectory.time_profile.travel_time_s);
  hash.number(trajectory.time_profile.translation_time_s);
  hash.number(trajectory.time_profile.stationary_turn_time_s);
  hash.value(
      static_cast<std::uint64_t>(trajectory.time_profile.arrival_times_s.size()));
  for (const double arrival_time_s : trajectory.time_profile.arrival_times_s) {
    hash.number(arrival_time_s);
  }
  hash.value(
      static_cast<std::uint64_t>(trajectory.time_profile.departure_times_s.size()));
  for (const double departure_time_s : trajectory.time_profile.departure_times_s) {
    hash.number(departure_time_s);
  }
  hash.value(trajectory.materialized_route_fingerprint);
  hash.value(trajectory.physical_route_fingerprint);
  return hash.result();
}

} // namespace

bool CompiledTrajectoryTimeProfile3D::valid() const noexcept {
  constexpr double kRelativeTolerance{1.0e-6};
  if (!std::isfinite(travel_time_s) || travel_time_s <= 0.0 ||
      !std::isfinite(translation_time_s) || translation_time_s <= 0.0 ||
      !std::isfinite(stationary_turn_time_s) || stationary_turn_time_s < 0.0 ||
      arrival_times_s.size() < 2U ||
      arrival_times_s.size() != departure_times_s.size()) {
    return false;
  }
  const double tolerance_s = kRelativeTolerance * std::max(1.0, travel_time_s);
  if (std::abs(travel_time_s - translation_time_s - stationary_turn_time_s) >
          tolerance_s ||
      std::abs(arrival_times_s.front()) > tolerance_s ||
      std::abs(departure_times_s.front()) > tolerance_s ||
      std::abs(arrival_times_s.back() - travel_time_s) > tolerance_s ||
      std::abs(departure_times_s.back() - travel_time_s) > tolerance_s) {
    return false;
  }

  double reconstructed_translation_time_s{0.0};
  double reconstructed_stationary_turn_time_s{0.0};
  for (std::size_t index = 0U; index < arrival_times_s.size(); ++index) {
    const double arrival_time_s = arrival_times_s[index];
    const double departure_time_s = departure_times_s[index];
    if (!std::isfinite(arrival_time_s) || !std::isfinite(departure_time_s) ||
        arrival_time_s < 0.0 || departure_time_s + tolerance_s < arrival_time_s) {
      return false;
    }
    reconstructed_stationary_turn_time_s += departure_time_s - arrival_time_s;
    if (index + 1U < arrival_times_s.size()) {
      const double next_arrival_time_s = arrival_times_s[index + 1U];
      if (!std::isfinite(next_arrival_time_s) ||
          next_arrival_time_s <= departure_time_s) {
        return false;
      }
      reconstructed_translation_time_s += next_arrival_time_s - departure_time_s;
    }
  }
  return std::abs(reconstructed_translation_time_s - translation_time_s) <=
             tolerance_s &&
         std::abs(reconstructed_stationary_turn_time_s - stationary_turn_time_s) <=
             tolerance_s;
}

CompiledTrajectory3D::CompiledTrajectory3D(
    VehicleState3D initial_state,
    const RouteEndpointSemantics3D compiled_endpoint_semantics,
    std::shared_ptr<const std::vector<RouteSample3D>> compiled_route,
    std::shared_ptr<const TrackingErrorTubeProfile3D> compiled_tracking_error_tube,
    std::shared_ptr<const std::vector<ConstrainedRouteSpan>> compiled_constrained_spans,
    CompiledTrajectoryTimeProfile3D compiled_time_profile,
    const std::uint64_t compiled_materialized_route_fingerprint,
    const std::uint64_t compiled_physical_route_fingerprint)
    : exact_initial_state{initial_state},
      endpoint_semantics{compiled_endpoint_semantics},
      route{std::move(compiled_route)},
      tracking_error_tube{std::move(compiled_tracking_error_tube)},
      constrained_spans{std::move(compiled_constrained_spans)},
      time_profile{std::move(compiled_time_profile)},
      materialized_route_fingerprint{compiled_materialized_route_fingerprint},
      physical_route_fingerprint{compiled_physical_route_fingerprint},
      compiled_trajectory_revision{calculateCompiledTrajectoryRevision(*this)} {
}

std::uint64_t
compiledTrajectoryRevision3D(const CompiledTrajectory3D& trajectory) noexcept {
  // Sealed at construction over immutable, non-copyable resources: the stored
  // seal is the exact recomputation, so re-hashing every sample on each
  // validity check would only repeat it.
  return trajectory.compiled_trajectory_revision;
}

} // namespace drone_city_nav
