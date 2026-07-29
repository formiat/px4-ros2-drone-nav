#include "drone_city_nav/active_global_guide.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr double kEpsilon{1.0e-9};

struct GlobalGuideRiskEvaluation {
  GlobalGuideRiskTier risk{GlobalGuideRiskTier::kPreferred};
  GlobalGuideAcceptanceReason rejection_reason{GlobalGuideAcceptanceReason::kAccepted};
};

[[nodiscard]] double dot(const Point2 first, const Point2 second) noexcept {
  return first.x * second.x + first.y * second.y;
}

[[nodiscard]] Point2 subtract(const Point2 first, const Point2 second) noexcept {
  return Point2{first.x - second.x, first.y - second.y};
}

[[nodiscard]] std::optional<float> clearanceAt(const mppi::EsdfGrid& grid,
                                               const std::span<const float> esdf_m,
                                               const Point2 point) {
  const int x =
      static_cast<int>(std::floor((point.x - grid.origin_x_m) / grid.resolution_m));
  const int y =
      static_cast<int>(std::floor((point.y - grid.origin_y_m) / grid.resolution_m));
  if (x < 0 || y < 0 || x >= grid.width || y >= grid.height) {
    return std::nullopt;
  }
  const std::size_t index =
      static_cast<std::size_t>(y) * static_cast<std::size_t>(grid.width) +
      static_cast<std::size_t>(x);
  if (index >= esdf_m.size()) {
    return std::nullopt;
  }
  return esdf_m[index];
}

[[nodiscard]] GlobalGuideRiskTier
riskTier(const float clearance_m, const ActiveGlobalGuideConfig& config) noexcept {
  if (std::isnan(clearance_m) ||
      (std::isinf(clearance_m) && std::signbit(clearance_m)) ||
      clearance_m <= config.collision_radius_m) {
    return GlobalGuideRiskTier::kCollision;
  }
  if (clearance_m < config.critical_distance_m) {
    return GlobalGuideRiskTier::kCritical;
  }
  if (clearance_m < config.preferred_distance_m) {
    return GlobalGuideRiskTier::kPlanning;
  }
  return GlobalGuideRiskTier::kPreferred;
}

[[nodiscard]] GlobalGuideRiskEvaluation
suffixRisk(const std::span<const Point2> guide, const double start_station_m,
           const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
           const ActiveGlobalGuideConfig& config) {
  if (guide.size() < 2U) {
    return {GlobalGuideRiskTier::kCollision,
            GlobalGuideAcceptanceReason::kInvalidGuide};
  }
  double total_length_m = 0.0;
  for (std::size_t index = 1U; index < guide.size(); ++index) {
    total_length_m += distance(guide[index - 1U], guide[index]);
  }
  if (!(total_length_m > kEpsilon)) {
    return {GlobalGuideRiskTier::kCollision,
            GlobalGuideAcceptanceReason::kInvalidGuide};
  }
  GlobalGuideRiskTier worst = GlobalGuideRiskTier::kPreferred;
  const auto evaluatePoint =
      [&](const Point2 point) -> std::optional<GlobalGuideAcceptanceReason> {
    const std::optional<float> clearance = clearanceAt(grid, esdf_m, point);
    if (!clearance.has_value()) {
      return GlobalGuideAcceptanceReason::kOutsideGrid;
    }
    if (std::isnan(*clearance) ||
        (std::isinf(*clearance) && std::signbit(*clearance))) {
      return GlobalGuideAcceptanceReason::kInvalidClearance;
    }
    const GlobalGuideRiskTier tier = riskTier(*clearance, config);
    worst = std::max(worst, tier);
    if (tier == GlobalGuideRiskTier::kCollision) {
      return GlobalGuideAcceptanceReason::kCollision;
    }
    return std::nullopt;
  };
  const double step_m = std::max(config.validation_sample_step_m,
                                 0.5 * static_cast<double>(grid.resolution_m));
  const double first_station = std::clamp(start_station_m, 0.0, total_length_m);
  const std::size_t sample_count =
      static_cast<std::size_t>(std::ceil((total_length_m - first_station) / step_m));
  for (std::size_t index = 0U; index < sample_count; ++index) {
    const double station = first_station + static_cast<double>(index) * step_m;
    if (const std::optional<GlobalGuideAcceptanceReason> rejection =
            evaluatePoint(sampleGlobalGuide(guide, station));
        rejection.has_value()) {
      return {GlobalGuideRiskTier::kCollision, *rejection};
    }
  }
  if (const std::optional<GlobalGuideAcceptanceReason> rejection =
          evaluatePoint(guide.back());
      rejection.has_value()) {
    return {GlobalGuideRiskTier::kCollision, *rejection};
  }
  return {worst, GlobalGuideAcceptanceReason::kAccepted};
}

[[nodiscard]] double blendAngles(const double first, const double second,
                                 const double ratio) noexcept {
  const double difference = std::remainder(second - first, 2.0 * std::numbers::pi);
  return std::remainder(first + std::clamp(ratio, 0.0, 1.0) * difference,
                        2.0 * std::numbers::pi);
}

} // namespace

GlobalGuideProjection projectOntoGlobalGuide(const std::span<const Point2> guide,
                                             const Point2 position,
                                             const double minimum_station_m) {
  GlobalGuideProjection result;
  if (guide.size() < 2U || !std::isfinite(position.x) || !std::isfinite(position.y)) {
    return result;
  }

  double cumulative_m = 0.0;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0U; index + 1U < guide.size(); ++index) {
    const Point2 segment = subtract(guide[index + 1U], guide[index]);
    const double segment_length_squared = dot(segment, segment);
    const double segment_length = std::sqrt(segment_length_squared);
    if (!(segment_length > kEpsilon)) {
      continue;
    }
    if (cumulative_m + segment_length + kEpsilon < minimum_station_m) {
      cumulative_m += segment_length;
      continue;
    }
    const double minimum_ratio =
        std::clamp((minimum_station_m - cumulative_m) / segment_length, 0.0, 1.0);
    const double projected_ratio =
        dot(subtract(position, guide[index]), segment) / segment_length_squared;
    const double ratio = std::clamp(projected_ratio, minimum_ratio, 1.0);
    const Point2 projected{guide[index].x + ratio * segment.x,
                           guide[index].y + ratio * segment.y};
    const double distance_squared = squaredDistance(position, projected);
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      result.valid = true;
      result.station_m = cumulative_m + ratio * segment_length;
      result.cross_track_m = std::sqrt(distance_squared);
      result.point = projected;
      result.tangent = Point2{segment.x / segment_length, segment.y / segment_length};
    }
    cumulative_m += segment_length;
  }
  result.total_length_m = cumulative_m;
  result.remaining_m = std::max(0.0, cumulative_m - result.station_m);
  return result;
}

Point2 sampleGlobalGuide(const std::span<const Point2> guide, const double station_m) {
  if (guide.empty()) {
    return {};
  }
  if (guide.size() == 1U || !(station_m > 0.0)) {
    return guide.front();
  }
  double cumulative_m = 0.0;
  for (std::size_t index = 0U; index + 1U < guide.size(); ++index) {
    const double segment_length = distance(guide[index], guide[index + 1U]);
    if (!(segment_length > kEpsilon)) {
      continue;
    }
    if (cumulative_m + segment_length >= station_m) {
      const double ratio =
          std::clamp((station_m - cumulative_m) / segment_length, 0.0, 1.0);
      return Point2{std::lerp(guide[index].x, guide[index + 1U].x, ratio),
                    std::lerp(guide[index].y, guide[index + 1U].y, ratio)};
    }
    cumulative_m += segment_length;
  }
  return guide.back();
}

ActiveGlobalGuideLifecycle::ActiveGlobalGuideLifecycle(
    const ActiveGlobalGuideConfig& config)
    : config_{config} {
  if (!(config_.collision_radius_m >= 0.0) ||
      !(config_.critical_distance_m > config_.collision_radius_m) ||
      !(config_.preferred_distance_m > config_.critical_distance_m) ||
      !(config_.validation_sample_step_m > 0.0) ||
      !(config_.minimum_remaining_m >= 0.0) || !(config_.maximum_cross_track_m > 0.0) ||
      !(config_.velocity_heading_low_speed_mps >= 0.0) ||
      !(config_.velocity_heading_high_speed_mps >
        config_.velocity_heading_low_speed_mps)) {
    throw std::invalid_argument{"invalid active global guide configuration"};
  }
}

ActiveGlobalGuideUpdate ActiveGlobalGuideLifecycle::update(
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point2 position, const std::uint64_t release_generation,
    const GlobalGuideReleaseReason release_reason) {
  status_ = {};
  status_.generation = generation_;
  if (!guide_) {
    consumed_stall_generation_ =
        std::max(consumed_stall_generation_, release_generation);
    status_.release_reason = GlobalGuideReleaseReason::kNoActiveGuide;
    return status_;
  }

  const GlobalGuideProjection projection =
      projectOntoGlobalGuide(*guide_, position, current_station_m_);
  status_.projection = projection;
  status_.generation = generation_;
  if (!projection.valid || projection.cross_track_m > config_.maximum_cross_track_m) {
    status_.release_reason = GlobalGuideReleaseReason::kDiverged;
  } else {
    current_station_m_ = std::max(current_station_m_, projection.station_m);
    reference_tangent_ = projection.tangent;
    reference_tangent_valid_ = true;
    status_.projection.station_m = current_station_m_;
    status_.projection.remaining_m =
        std::max(0.0, projection.total_length_m - current_station_m_);
    status_.current_risk =
        suffixRisk(*guide_, current_station_m_, grid, esdf_m, config_).risk;
    const bool newly_blocked = status_.current_risk == GlobalGuideRiskTier::kCollision;
    status_.reaches_mission_goal = reaches_mission_goal_;
    if (newly_blocked) {
      status_.release_reason = GlobalGuideReleaseReason::kBlocked;
    } else if (release_generation > consumed_stall_generation_) {
      consumed_stall_generation_ = release_generation;
      status_.release_reason =
          release_reason == GlobalGuideReleaseReason::kPersistentSafetyRejection
              ? GlobalGuideReleaseReason::kPersistentSafetyRejection
              : GlobalGuideReleaseReason::kStalled;
    } else if (!reaches_mission_goal_ &&
               status_.projection.remaining_m < config_.minimum_remaining_m) {
      status_.active = true;
      status_.retained = true;
      status_.requires_replan = true;
      status_.release_reason = GlobalGuideReleaseReason::kExhausted;
      return status_;
    } else {
      status_.active = true;
      status_.retained = true;
      status_.requires_replan = status_.current_risk == GlobalGuideRiskTier::kCritical;
      status_.release_reason = GlobalGuideReleaseReason::kNone;
      return status_;
    }
  }

  if (status_.release_reason == GlobalGuideReleaseReason::kBlocked ||
      status_.release_reason == GlobalGuideReleaseReason::kDiverged) {
    reference_tangent_valid_ = false;
  }
  guide_.reset();
  reaches_mission_goal_ = false;
  current_station_m_ = 0.0;
  return status_;
}

GlobalGuideAcceptanceResult ActiveGlobalGuideLifecycle::accept(
    std::shared_ptr<const std::vector<Point2>> guide, const bool reaches_mission_goal,
    const mppi::EsdfGrid& grid, const std::span<const float> esdf_m,
    const Point2 position) {
  if (!guide || guide->size() < 2U) {
    return {.reason = GlobalGuideAcceptanceReason::kInvalidGuide,
            .risk = GlobalGuideRiskTier::kCollision};
  }
  const GlobalGuideProjection projection = projectOntoGlobalGuide(*guide, position);
  if (!projection.valid) {
    return {.reason = GlobalGuideAcceptanceReason::kInvalidProjection,
            .risk = GlobalGuideRiskTier::kCollision,
            .projection = projection};
  }
  if (projection.cross_track_m > config_.maximum_cross_track_m) {
    return {.reason = GlobalGuideAcceptanceReason::kCrossTrackExceeded,
            .risk = GlobalGuideRiskTier::kCollision,
            .projection = projection};
  }
  const GlobalGuideRiskEvaluation evaluation =
      suffixRisk(*guide, projection.station_m, grid, esdf_m, config_);
  if (evaluation.rejection_reason != GlobalGuideAcceptanceReason::kAccepted) {
    return {.reason = evaluation.rejection_reason,
            .risk = evaluation.risk,
            .projection = projection};
  }

  guide_ = std::move(guide);
  ++generation_;
  current_station_m_ = projection.station_m;
  reaches_mission_goal_ = reaches_mission_goal;
  accepted_risk_ = evaluation.risk;
  reference_tangent_ = projection.tangent;
  reference_tangent_valid_ = true;
  status_ = ActiveGlobalGuideUpdate{
      .active = true,
      .retained = false,
      .requires_replan = false,
      .reaches_mission_goal = reaches_mission_goal_,
      .generation = generation_,
      .release_reason = GlobalGuideReleaseReason::kNone,
      .current_risk = accepted_risk_,
      .projection = projection,
  };
  return {.accepted = true,
          .reason = GlobalGuideAcceptanceReason::kAccepted,
          .risk = accepted_risk_,
          .projection = projection};
}

GlobalGuideHeading
ActiveGlobalGuideLifecycle::selectPlanningHeading(const mppi::State& state,
                                                  const double yaw_fallback_rad) const {
  const double speed_mps =
      std::hypot(static_cast<double>(state.vx), static_cast<double>(state.vy));
  const double velocity_heading =
      std::atan2(static_cast<double>(state.vy), static_cast<double>(state.vx));
  const double guide_heading = std::atan2(reference_tangent_.y, reference_tangent_.x);
  if (speed_mps >= config_.velocity_heading_high_speed_mps) {
    return GlobalGuideHeading{velocity_heading, GlobalGuideHeadingSource::kVelocity};
  }
  if (reference_tangent_valid_ && speed_mps <= config_.velocity_heading_low_speed_mps) {
    return GlobalGuideHeading{guide_heading, GlobalGuideHeadingSource::kActiveGuide};
  }
  if (reference_tangent_valid_ && speed_mps > config_.velocity_heading_low_speed_mps) {
    const double ratio = (speed_mps - config_.velocity_heading_low_speed_mps) /
                         (config_.velocity_heading_high_speed_mps -
                          config_.velocity_heading_low_speed_mps);
    return GlobalGuideHeading{blendAngles(guide_heading, velocity_heading, ratio),
                              GlobalGuideHeadingSource::kBlended};
  }
  if (speed_mps > config_.velocity_heading_low_speed_mps) {
    return GlobalGuideHeading{velocity_heading, GlobalGuideHeadingSource::kVelocity};
  }
  return GlobalGuideHeading{std::isfinite(yaw_fallback_rad) ? yaw_fallback_rad : 0.0,
                            GlobalGuideHeadingSource::kYawFallback};
}

std::shared_ptr<const std::vector<Point2>>
ActiveGlobalGuideLifecycle::guide() const noexcept {
  return guide_;
}

ActiveGlobalGuideUpdate ActiveGlobalGuideLifecycle::status() const noexcept {
  return status_;
}

GlobalGuideProgressTracker::GlobalGuideProgressTracker(
    const GlobalGuideProgressConfig& config)
    : config_{config} {
  if (!(config_.observation_window_s > 0.0) || !(config_.minimum_progress_m >= 0.0) ||
      !(config_.minimum_predicted_head_progress_m >= 0.0) ||
      !(config_.persistent_safety_rejection_window_s > 0.0)) {
    throw std::invalid_argument{"invalid global guide progress configuration"};
  }
}

GlobalGuideProgressUpdate GlobalGuideProgressTracker::evaluate(
    const GlobalGuideProgressObservation& observation) {
  GlobalGuideProgressUpdate update{.stall_generation = stall_generation_};
  if (observation.stamp_ns <= 0 || observation.guide_generation == 0U ||
      !std::isfinite(observation.station_m) ||
      !std::isfinite(observation.predicted_head_progress_m) ||
      !observation.controller_active) {
    anchor_valid_ = false;
    safety_rejection_anchor_valid_ = false;
    return update;
  }
  if (observation.emergency_braking) {
    if (!safety_rejection_anchor_valid_ ||
        safety_rejection_guide_generation_ != observation.guide_generation ||
        observation.stamp_ns < safety_rejection_anchor_stamp_ns_) {
      safety_rejection_anchor_valid_ = true;
      safety_rejection_anchor_stamp_ns_ = observation.stamp_ns;
      safety_rejection_guide_generation_ = observation.guide_generation;
      return update;
    }
    update.observation_age_s =
        static_cast<double>(observation.stamp_ns - safety_rejection_anchor_stamp_ns_) /
        1.0e9;
    if (update.observation_age_s < config_.persistent_safety_rejection_window_s) {
      return update;
    }
    ++stall_generation_;
    update.stalled = true;
    update.persistent_safety_rejection = true;
    update.stall_generation = stall_generation_;
    safety_rejection_anchor_stamp_ns_ = observation.stamp_ns;
    anchor_valid_ = false;
    return update;
  }
  safety_rejection_anchor_valid_ = false;
  if (!anchor_valid_ || observation.guide_generation != anchor_guide_generation_ ||
      observation.stamp_ns < anchor_stamp_ns_) {
    resetAnchor(observation);
    return update;
  }

  update.observation_age_s =
      static_cast<double>(observation.stamp_ns - anchor_stamp_ns_) / 1.0e9;
  update.progress_m = observation.station_m - anchor_station_m_;
  if (update.progress_m >= config_.minimum_progress_m) {
    resetAnchor(observation);
    return update;
  }
  if (update.observation_age_s < config_.observation_window_s ||
      observation.predicted_head_progress_m <
          config_.minimum_predicted_head_progress_m) {
    return update;
  }

  ++stall_generation_;
  update.stalled = true;
  update.stall_generation = stall_generation_;
  resetAnchor(observation);
  return update;
}

void GlobalGuideProgressTracker::resetAnchor(
    const GlobalGuideProgressObservation& observation) noexcept {
  anchor_valid_ = true;
  anchor_stamp_ns_ = observation.stamp_ns;
  anchor_guide_generation_ = observation.guide_generation;
  anchor_station_m_ = observation.station_m;
}

const char*
globalGuideReleaseReasonName(const GlobalGuideReleaseReason reason) noexcept {
  switch (reason) {
    case GlobalGuideReleaseReason::kNone:
      return "none";
    case GlobalGuideReleaseReason::kNoActiveGuide:
      return "no_active_guide";
    case GlobalGuideReleaseReason::kBlocked:
      return "blocked";
    case GlobalGuideReleaseReason::kExhausted:
      return "exhausted";
    case GlobalGuideReleaseReason::kStalled:
      return "stalled";
    case GlobalGuideReleaseReason::kPersistentSafetyRejection:
      return "persistent_safety_rejection";
    case GlobalGuideReleaseReason::kDiverged:
      return "diverged";
  }
  return "unknown";
}

const char*
globalGuideHeadingSourceName(const GlobalGuideHeadingSource source) noexcept {
  switch (source) {
    case GlobalGuideHeadingSource::kVelocity:
      return "velocity";
    case GlobalGuideHeadingSource::kActiveGuide:
      return "active_guide";
    case GlobalGuideHeadingSource::kBlended:
      return "blended";
    case GlobalGuideHeadingSource::kYawFallback:
      return "yaw_fallback";
  }
  return "unknown";
}

const char* globalGuideRiskTierName(const GlobalGuideRiskTier tier) noexcept {
  switch (tier) {
    case GlobalGuideRiskTier::kPreferred:
      return "preferred";
    case GlobalGuideRiskTier::kPlanning:
      return "planning";
    case GlobalGuideRiskTier::kCritical:
      return "critical";
    case GlobalGuideRiskTier::kCollision:
      return "collision";
  }
  return "unknown";
}

const char*
globalGuideAcceptanceReasonName(const GlobalGuideAcceptanceReason reason) noexcept {
  switch (reason) {
    case GlobalGuideAcceptanceReason::kNotAttempted:
      return "not_attempted";
    case GlobalGuideAcceptanceReason::kAccepted:
      return "accepted";
    case GlobalGuideAcceptanceReason::kInvalidGuide:
      return "invalid_guide";
    case GlobalGuideAcceptanceReason::kInvalidProjection:
      return "invalid_projection";
    case GlobalGuideAcceptanceReason::kCrossTrackExceeded:
      return "cross_track_exceeded";
    case GlobalGuideAcceptanceReason::kOutsideGrid:
      return "outside_grid";
    case GlobalGuideAcceptanceReason::kInvalidClearance:
      return "invalid_clearance";
    case GlobalGuideAcceptanceReason::kCollision:
      return "collision";
  }
  return "unknown";
}

} // namespace drone_city_nav
