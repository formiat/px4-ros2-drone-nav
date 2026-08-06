#include "drone_city_nav/tracking_objective.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] Point3 interpolate(const Point3& first, const Point3& second,
                                 const double fraction) noexcept {
  return Point3{first.x + fraction * (second.x - first.x),
                first.y + fraction * (second.y - first.y),
                first.z + fraction * (second.z - first.z)};
}

using RawOccupiedQuery = std::function<bool(const Point3&)>;
using SweptClearQuery = std::function<bool(const Point3&, const Point3&)>;

constexpr std::size_t kDirectTargetRefinementIterations{8U};

[[nodiscard]] TrackingObjectiveResolution
resolve(const Point3& observed_position, const Point3& predicted_position,
        const double segment_length_m, const double grid_resolution_m,
        const double maximum_sample_spacing_m, const RawOccupiedQuery& raw_occupied) {
  if (!finite(observed_position) || !finite(predicted_position) ||
      !(grid_resolution_m > 0.0) || !(maximum_sample_spacing_m > 0.0)) {
    return {};
  }
  if (segment_length_m <= 1.0e-9) {
    return TrackingObjectiveResolution{
        .resolved_position = predicted_position,
        .status = TrackingObjectiveResolutionStatus::kUnchanged,
        .resolved_fraction = 1.0,
    };
  }

  const double sample_spacing_m =
      std::min(maximum_sample_spacing_m, grid_resolution_m * 0.5);
  const auto sample_count = static_cast<std::size_t>(
      std::max(1.0, std::ceil(segment_length_m / sample_spacing_m)));
  Point3 last_free = observed_position;
  double last_free_fraction = 0.0;
  for (std::size_t sample = 1U; sample <= sample_count; ++sample) {
    const double fraction =
        static_cast<double>(sample) / static_cast<double>(sample_count);
    const Point3 point = interpolate(observed_position, predicted_position, fraction);
    if (raw_occupied(point)) {
      return TrackingObjectiveResolution{
          .resolved_position = last_free,
          .status = last_free_fraction > 0.0
                        ? TrackingObjectiveResolutionStatus::kClippedRawOccupied
                        : TrackingObjectiveResolutionStatus::kFallbackObserved,
          .resolved_fraction = last_free_fraction,
      };
    }
    last_free = point;
    last_free_fraction = fraction;
  }

  return TrackingObjectiveResolution{
      .resolved_position = predicted_position,
      .status = TrackingObjectiveResolutionStatus::kUnchanged,
      .resolved_fraction = 1.0,
  };
}

[[nodiscard]] DirectTrackingTargetResolution
resolveDirectTarget(const Point3& interceptor_position,
                    const Point3& current_target_position,
                    const Point3& predicted_target_position,
                    const TrackingObjectiveResolution& prediction_resolution,
                    const SweptClearQuery& swept_clear) {
  DirectTrackingTargetResolution result{
      .selected_position = current_target_position,
      .status = DirectTrackingTargetStatus::kInvalidInput,
  };
  if (!finite(interceptor_position) || !finite(current_target_position) ||
      !finite(predicted_target_position) ||
      prediction_resolution.status ==
          TrackingObjectiveResolutionStatus::kInvalidInput) {
    return result;
  }

  result.observed_target_visible =
      swept_clear(interceptor_position, current_target_position);
  if (!result.observed_target_visible) {
    result.status = DirectTrackingTargetStatus::kObservedTargetOccluded;
    return result;
  }

  const double maximum_fraction =
      std::clamp(prediction_resolution.resolved_fraction, 0.0, 1.0);
  const bool full_prediction_raw_clear =
      prediction_resolution.status == TrackingObjectiveResolutionStatus::kUnchanged &&
      maximum_fraction >= 1.0 - 1.0e-9;
  if (full_prediction_raw_clear &&
      swept_clear(interceptor_position, predicted_target_position)) {
    result.selected_position = predicted_target_position;
    result.selected_prediction_fraction = 1.0;
    result.status = DirectTrackingTargetStatus::kFullPrediction;
    result.predicted_intercept_path_clear = true;
    return result;
  }

  double clear_fraction = 0.0;
  double blocked_fraction = maximum_fraction;
  const Point3 maximum_candidate =
      interpolate(current_target_position, predicted_target_position, maximum_fraction);
  if (maximum_fraction > 0.0 && swept_clear(interceptor_position, maximum_candidate)) {
    clear_fraction = maximum_fraction;
  } else {
    for (std::size_t iteration = 0U; iteration < kDirectTargetRefinementIterations;
         ++iteration) {
      const double candidate_fraction = 0.5 * (clear_fraction + blocked_fraction);
      const Point3 candidate = interpolate(
          current_target_position, predicted_target_position, candidate_fraction);
      if (swept_clear(interceptor_position, candidate)) {
        clear_fraction = candidate_fraction;
      } else {
        blocked_fraction = candidate_fraction;
      }
    }
  }

  result.selected_prediction_fraction = clear_fraction;
  result.selected_position =
      interpolate(current_target_position, predicted_target_position, clear_fraction);
  result.status = clear_fraction > 1.0e-6
                      ? DirectTrackingTargetStatus::kShortenedPrediction
                      : DirectTrackingTargetStatus::kCurrentTargetOnly;
  return result;
}

} // namespace

TrackingLineOfSightLifecycle::TrackingLineOfSightLifecycle(
    const TrackingLineOfSightConfig& config)
    : config_{config} {
  if (config_.clear_confirmations == 0U) {
    throw std::invalid_argument{"tracking LOS confirmations must be positive"};
  }
}

TrackingLineOfSightUpdate
TrackingLineOfSightLifecycle::update(const bool raw_clear) noexcept {
  TrackingLineOfSightUpdate result{.active = active_, .generation = generation_};
  if (!raw_clear) {
    clear_count_ = 0U;
    if (active_) {
      active_ = false;
      result.active = false;
      result.newly_inactive = true;
    }
    return result;
  }
  if (active_) {
    return result;
  }
  ++clear_count_;
  if (clear_count_ >= config_.clear_confirmations) {
    active_ = true;
    clear_count_ = config_.clear_confirmations;
    ++generation_;
    result.active = true;
    result.newly_active = true;
    result.generation = generation_;
  }
  return result;
}

void TrackingLineOfSightLifecycle::reset() noexcept {
  clear_count_ = 0U;
  active_ = false;
}

TrackingObjectiveResolution resolveTrackingObjective(
    const OccupancyGrid2D& raw_occupancy, const Point3& observed_position,
    const Point3& predicted_position, const double maximum_sample_spacing_m) {
  const double segment_length_m =
      std::hypot(predicted_position.x - observed_position.x,
                 predicted_position.y - observed_position.y);
  return resolve(observed_position, predicted_position, segment_length_m,
                 raw_occupancy.resolution(), maximum_sample_spacing_m,
                 [&raw_occupancy](const Point3& point) {
                   const auto cell =
                       raw_occupancy.worldToCell(Point2{point.x, point.y});
                   return cell.has_value() && raw_occupancy.isOccupied(*cell);
                 });
}

bool trackingLineOfSightRawClear(const OccupancyGrid2D& raw_occupancy,
                                 const Point3& from, const Point3& to,
                                 const double maximum_sample_spacing_m) {
  return resolveTrackingObjective(raw_occupancy, from, to, maximum_sample_spacing_m)
             .status == TrackingObjectiveResolutionStatus::kUnchanged;
}

bool trackingLineOfSightRawClear(const OccupancyGrid3D& raw_occupancy,
                                 const Point3& from, const Point3& to,
                                 const double maximum_sample_spacing_m) {
  return resolveTrackingObjective(raw_occupancy, from, to, maximum_sample_spacing_m)
             .status == TrackingObjectiveResolutionStatus::kUnchanged;
}

bool trackingLineOfSightSweptRawClear(const OccupancyGrid2D& raw_occupancy,
                                      const Point3& from, const Point3& to,
                                      const SweptFootprintConfig& footprint) {
  return validateRawSweptFootprint(raw_occupancy, from, to, footprint).accepted();
}

bool trackingLineOfSightSweptRawClear(const OccupancyGrid3D& raw_occupancy,
                                      const Point3& from, const Point3& to,
                                      const SweptFootprintConfig& footprint) {
  return validateRawSweptFootprint(raw_occupancy, from, FootprintBodyAxis{}, to,
                                   FootprintBodyAxis{}, footprint)
      .accepted();
}

DirectTrackingTargetResolution resolveDirectTrackingTarget(
    const OccupancyGrid2D& raw_occupancy, const Point3& interceptor_position,
    const Point3& current_target_position, const Point3& predicted_target_position,
    const SweptFootprintConfig& footprint) {
  const TrackingObjectiveResolution prediction_resolution =
      resolveTrackingObjective(raw_occupancy, current_target_position,
                               predicted_target_position, footprint.sweep_step_m);
  return resolveDirectTarget(
      interceptor_position, current_target_position, predicted_target_position,
      prediction_resolution,
      [&raw_occupancy, &footprint](const Point3& from, const Point3& to) {
        return trackingLineOfSightSweptRawClear(raw_occupancy, from, to, footprint);
      });
}

DirectTrackingTargetResolution resolveDirectTrackingTarget(
    const OccupancyGrid3D& raw_occupancy, const Point3& interceptor_position,
    const Point3& current_target_position, const Point3& predicted_target_position,
    const SweptFootprintConfig& footprint) {
  const TrackingObjectiveResolution prediction_resolution =
      resolveTrackingObjective(raw_occupancy, current_target_position,
                               predicted_target_position, footprint.sweep_step_m);
  return resolveDirectTarget(
      interceptor_position, current_target_position, predicted_target_position,
      prediction_resolution,
      [&raw_occupancy, &footprint](const Point3& from, const Point3& to) {
        return trackingLineOfSightSweptRawClear(raw_occupancy, from, to, footprint);
      });
}

TrackingObjectiveResolution resolveTrackingObjective(
    const OccupancyGrid3D& raw_occupancy, const Point3& observed_position,
    const Point3& predicted_position, const double maximum_sample_spacing_m) {
  return resolve(observed_position, predicted_position,
                 distance3D(observed_position, predicted_position),
                 raw_occupancy.bounds().resolution_m, maximum_sample_spacing_m,
                 [&raw_occupancy](const Point3& point) {
                   const auto cell = raw_occupancy.worldToCell(point);
                   return cell.has_value() && raw_occupancy.isOccupied(*cell);
                 });
}

const char* trackingObjectiveResolutionStatusName(
    const TrackingObjectiveResolutionStatus status) noexcept {
  switch (status) {
    case TrackingObjectiveResolutionStatus::kUnchanged:
      return "unchanged";
    case TrackingObjectiveResolutionStatus::kClippedRawOccupied:
      return "clipped_raw_occupied";
    case TrackingObjectiveResolutionStatus::kFallbackObserved:
      return "fallback_observed";
    case TrackingObjectiveResolutionStatus::kWorldUnavailable:
      return "world_unavailable";
    case TrackingObjectiveResolutionStatus::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

const char*
directTrackingTargetStatusName(const DirectTrackingTargetStatus status) noexcept {
  switch (status) {
    case DirectTrackingTargetStatus::kFullPrediction:
      return "full_prediction";
    case DirectTrackingTargetStatus::kShortenedPrediction:
      return "shortened_prediction";
    case DirectTrackingTargetStatus::kCurrentTargetOnly:
      return "current_target_only";
    case DirectTrackingTargetStatus::kObservedTargetOccluded:
      return "observed_target_occluded";
    case DirectTrackingTargetStatus::kWorldUnavailable:
      return "world_unavailable";
    case DirectTrackingTargetStatus::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

} // namespace drone_city_nav
