#include "drone_city_nav/passage_coordinator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace drone_city_nav {
namespace {

constexpr double kEpsilon{1.0e-9};

struct PassageCoordinates {
  double longitudinal_m{0.0};
  double lateral_m{0.0};
};

[[nodiscard]] PassageCoordinates passageCoordinates(const mppi::State& state,
                                                    const PassageOpening& opening,
                                                    const double travel_sign) noexcept {
  const double dx = static_cast<double>(state.x) - opening.center.x;
  const double dy = static_cast<double>(state.y) - opening.center.y;
  return PassageCoordinates{
      travel_sign * (dx * opening.normal_xy.x + dy * opening.normal_xy.y),
      -dx * opening.normal_xy.y + dy * opening.normal_xy.x,
  };
}

[[nodiscard]] bool inside(const double value, const double lower,
                          const double upper) noexcept {
  return value >= lower && value <= upper;
}

[[nodiscard]] double distanceToInterval(const double value, const double lower,
                                        const double upper) noexcept {
  if (value < lower) {
    return lower - value;
  }
  if (value > upper) {
    return value - upper;
  }
  return 0.0;
}

[[nodiscard]] double minimumRestToRestTime(const double distance_m,
                                           const double acceleration_mps2,
                                           const double speed_mps) noexcept {
  if (!(distance_m > 0.0)) {
    return 0.0;
  }
  const double acceleration_distance = speed_mps * speed_mps / acceleration_mps2;
  if (distance_m <= acceleration_distance) {
    return 2.0 * std::sqrt(distance_m / acceleration_mps2);
  }
  return 2.0 * speed_mps / acceleration_mps2 +
         (distance_m - acceleration_distance) / speed_mps;
}

[[nodiscard]] bool sameOpening(const PassageOpening* selected,
                               const PassageOpening& active) noexcept {
  return selected != nullptr && selected->id == active.id &&
         selected->structure_id == active.structure_id;
}

[[nodiscard]] double selectTravelSign(const double longitudinal,
                                      const double velocity_along_normal) noexcept {
  if (std::abs(longitudinal) > kEpsilon) {
    return longitudinal < 0.0 ? 1.0 : -1.0;
  }
  return velocity_along_normal >= 0.0 ? 1.0 : -1.0;
}

[[nodiscard]] mppi::PassagePhase
toMppiPassagePhase(const PassageCoordinatorPhase phase) noexcept {
  switch (phase) {
    case PassageCoordinatorPhase::kPartialFromInside:
      return mppi::PassagePhase::kPartialFromInside;
    case PassageCoordinatorPhase::kTraversal:
      return mppi::PassagePhase::kTraversal;
    case PassageCoordinatorPhase::kStationaryVerticalAlignment:
      return mppi::PassagePhase::kStationaryVerticalAlignment;
    case PassageCoordinatorPhase::kInactive:
    case PassageCoordinatorPhase::kApproach:
    case PassageCoordinatorPhase::kInvalidOpening:
      return mppi::PassagePhase::kApproach;
  }
  return mppi::PassagePhase::kApproach;
}

} // namespace

PassageCoordinator::PassageCoordinator(const PassageCoordinatorConfig& config)
    : config_{config} {
  if (!(config_.vertical_clearance_margin_m >= 0.0) ||
      !(config_.vertical_capture_hysteresis_m >= 0.0) ||
      !(config_.preferred_z_capture_tolerance_m > 0.0) ||
      !(config_.maximum_capture_vertical_speed_mps >= 0.0) ||
      config_.capture_stable_cycles == 0U || config_.retention_violation_cycles == 0U ||
      !(config_.lateral_alignment_tolerance_m > 0.0) ||
      !(config_.approach_alignment_speed_mps > 0.0) ||
      !(config_.approach_staging_distance_m >= 0.0) ||
      !(config_.alignment_time_margin_s >= 0.0) ||
      !(config_.minimum_stationary_trigger_distance_m >= 0.0) ||
      !(config_.maximum_vertical_acceleration_mps2 > 0.0) ||
      !(config_.maximum_vertical_speed_mps > 0.0) ||
      !(config_.maximum_horizontal_braking_acceleration_mps2 > 0.0) ||
      !(config_.reaction_latency_s >= 0.0)) {
    throw std::invalid_argument{"invalid passage coordinator configuration"};
  }
}

PassageCoordinatorResult
PassageCoordinator::update(const PassageCoordinatorInput& input) {
  if (!active_opening_.has_value() && input.selected_opening == nullptr) {
    return {};
  }
  if (!active_opening_.has_value()) {
    active_opening_ = *input.selected_opening;
    const double dx = static_cast<double>(input.state.x) - active_opening_->center.x;
    const double dy = static_cast<double>(input.state.y) - active_opening_->center.y;
    const double raw_longitudinal =
        dx * active_opening_->normal_xy.x + dy * active_opening_->normal_xy.y;
    const double velocity_along_normal =
        static_cast<double>(input.state.vx) * active_opening_->normal_xy.x +
        static_cast<double>(input.state.vy) * active_opening_->normal_xy.y;
    travel_sign_ = selectTravelSign(raw_longitudinal, velocity_along_normal);
    preferred_z_m_ = 0.5 * (active_opening_->min_z_m + active_opening_->max_z_m);
    const PassageCoordinates coordinates =
        passageCoordinates(input.state, *active_opening_, travel_sign_);
    const bool inside_footprint =
        std::abs(coordinates.longitudinal_m) <= 0.5 * active_opening_->depth_m &&
        std::abs(coordinates.lateral_m) <= 0.5 * active_opening_->width_m;
    const double safe_min_z =
        active_opening_->min_z_m + config_.vertical_clearance_margin_m;
    const double safe_max_z =
        active_opening_->max_z_m - config_.vertical_clearance_margin_m;
    if (inside_footprint &&
        inside(static_cast<double>(input.state.z), safe_min_z, safe_max_z)) {
      partial_from_inside_ = true;
      vertical_ready_latched_ = true;
      traversal_latched_ = true;
      preferred_z_m_ = input.state.z;
    }
  }

  const PassageOpening& opening = *active_opening_;
  const double safe_min_z = opening.min_z_m + config_.vertical_clearance_margin_m;
  const double safe_max_z = opening.max_z_m - config_.vertical_clearance_margin_m;
  const double capture_min_z =
      std::max(safe_min_z + config_.vertical_capture_hysteresis_m,
               preferred_z_m_ - config_.preferred_z_capture_tolerance_m);
  const double capture_max_z =
      std::min(safe_max_z - config_.vertical_capture_hysteresis_m,
               preferred_z_m_ + config_.preferred_z_capture_tolerance_m);
  const double half_width_m = 0.5 * opening.width_m;
  if (!(capture_max_z > capture_min_z) || !(half_width_m > 0.0)) {
    PassageCoordinatorResult invalid{
        .phase = PassageCoordinatorPhase::kInvalidOpening,
        .constraint = std::nullopt,
        .opening_id = opening.id,
    };
    reset();
    return invalid;
  }

  const PassageCoordinates coordinates =
      passageCoordinates(input.state, opening, travel_sign_);
  const double half_depth_m = 0.5 * opening.depth_m;
  const double exit_boundary_m = half_depth_m + opening.exit_distance_m;
  bool cleared_exit = coordinates.longitudinal_m > exit_boundary_m;
  if (partial_from_inside_) {
    cleared_exit = std::abs(coordinates.longitudinal_m) > exit_boundary_m ||
                   std::abs(coordinates.lateral_m) > half_width_m;
  }
  if (cleared_exit) {
    reset();
    return {};
  }
  if (!sameOpening(input.selected_opening, opening) && !traversal_latched_ &&
      !vertical_alignment_active_ &&
      std::abs(coordinates.lateral_m) > 0.5 * opening.width_m) {
    reset();
    return {};
  }

  const double z_m = input.state.z;
  const bool inside_capture_window = inside(z_m, capture_min_z, capture_max_z);
  const bool inside_retention_window = inside(z_m, safe_min_z, safe_max_z);
  const bool capture_velocity_safe = std::abs(static_cast<double>(input.state.vz)) <=
                                     config_.maximum_capture_vertical_speed_mps;
  const bool instantaneously_captured = inside_capture_window && capture_velocity_safe;
  if (instantaneously_captured) {
    capture_stable_cycles_ =
        std::min(capture_stable_cycles_ + 1U, config_.capture_stable_cycles);
  } else {
    capture_stable_cycles_ = 0U;
  }
  const bool captured = capture_stable_cycles_ >= config_.capture_stable_cycles;
  if (captured) {
    vertical_ready_latched_ = true;
  }
  const double vertical_error_m = distanceToInterval(z_m, capture_min_z, capture_max_z);
  const double braking_time_s = std::abs(static_cast<double>(input.state.vz)) /
                                config_.maximum_vertical_acceleration_mps2;
  const double alignment_time_s =
      braking_time_s + minimumRestToRestTime(vertical_error_m,
                                             config_.maximum_vertical_acceleration_mps2,
                                             config_.maximum_vertical_speed_mps);
  const double approach_speed_mps =
      std::max(0.0, std::min(input.approach_speed_mps, input.passage_speed_limit_mps));
  const double vertical_alignment_distance_m =
      approach_speed_mps * (alignment_time_s + config_.alignment_time_margin_s);
  const double horizontal_speed_mps = std::hypot(static_cast<double>(input.state.vx),
                                                 static_cast<double>(input.state.vy));
  const double stopping_distance_m =
      horizontal_speed_mps * config_.reaction_latency_s +
      horizontal_speed_mps * horizontal_speed_mps /
          (2.0 * config_.maximum_horizontal_braking_acceleration_mps2);
  const double required_distance_m =
      std::max(vertical_alignment_distance_m, stopping_distance_m);
  const double distance_to_entry_m =
      std::max(0.0, -half_depth_m - coordinates.longitudinal_m);
  const double lateral_error_m = std::abs(coordinates.lateral_m);
  const double lateral_alignment_limit_m =
      std::min(config_.lateral_alignment_tolerance_m, half_width_m);
  const bool lateral_aligned = lateral_error_m <= lateral_alignment_limit_m;
  const double stationary_trigger_distance_m =
      std::max(config_.minimum_stationary_trigger_distance_m, required_distance_m);
  const double staging_offset_m = std::max(
      opening.approach_distance_m, half_depth_m + config_.approach_staging_distance_m);

  if (vertical_alignment_active_) {
    if (vertical_ready_latched_) {
      vertical_alignment_active_ = false;
      traversal_latched_ = lateral_aligned;
    }
  } else if (!partial_from_inside_ && !traversal_latched_ && !vertical_ready_latched_ &&
             distance_to_entry_m <= stationary_trigger_distance_m) {
    vertical_alignment_active_ = true;
    hold_position_ = Point2{input.state.x, input.state.y};
  } else if (!partial_from_inside_ && !traversal_latched_ && vertical_ready_latched_ &&
             lateral_aligned) {
    traversal_latched_ = true;
  }
  const bool staging_required = !partial_from_inside_ && !vertical_alignment_active_ &&
                                !traversal_latched_ && vertical_ready_latched_ &&
                                !lateral_aligned;
  if (staging_required && !approach_target_.has_value()) {
    const double target_longitudinal_m =
        std::min(coordinates.longitudinal_m, -staging_offset_m);
    approach_target_ = Point2{
        opening.center.x + travel_sign_ * opening.normal_xy.x * target_longitudinal_m,
        opening.center.y + travel_sign_ * opening.normal_xy.y * target_longitudinal_m,
    };
  }

  PassageCoordinatorPhase phase = PassageCoordinatorPhase::kApproach;
  if (partial_from_inside_) {
    phase = PassageCoordinatorPhase::kPartialFromInside;
  } else if (vertical_alignment_active_) {
    phase = PassageCoordinatorPhase::kStationaryVerticalAlignment;
  } else if (traversal_latched_) {
    phase = PassageCoordinatorPhase::kTraversal;
  }

  if (vertical_ready_latched_ && !partial_from_inside_) {
    if (inside_retention_window) {
      retention_violation_cycles_ = 0U;
    } else {
      retention_violation_cycles_ = std::min(retention_violation_cycles_ + 1U,
                                             config_.retention_violation_cycles);
    }
    if (retention_violation_cycles_ >= config_.retention_violation_cycles) {
      vertical_alignment_active_ = true;
      vertical_ready_latched_ = false;
      traversal_latched_ = false;
      capture_stable_cycles_ = 0U;
      retention_violation_cycles_ = 0U;
      hold_position_ = Point2{input.state.x, input.state.y};
      phase = PassageCoordinatorPhase::kStationaryVerticalAlignment;
    }
  } else {
    retention_violation_cycles_ = 0U;
  }

  const mppi::PassagePhase mppi_phase = toMppiPassagePhase(phase);
  PassageCoordinatorResult result{
      .phase = phase,
      .constraint =
          mppi::PassageConstraint{
              static_cast<float>(opening.center.x),
              static_cast<float>(opening.center.y),
              static_cast<float>(travel_sign_ * opening.normal_xy.x),
              static_cast<float>(travel_sign_ * opening.normal_xy.y),
              static_cast<float>(half_depth_m),
              static_cast<float>(safe_min_z),
              static_cast<float>(safe_max_z),
              static_cast<float>(preferred_z_m_),
              static_cast<float>(opening.approach_distance_m),
              static_cast<float>(opening.exit_distance_m),
              static_cast<float>(input.passage_speed_limit_mps),
              mppi_phase,
          },
      .opening_id = opening.id,
      .active = true,
      .hold_xy = vertical_alignment_active_,
      .approach_alignment_active = staging_required,
      .vertical_ready = partial_from_inside_ || vertical_ready_latched_,
      .hold_position = hold_position_,
      .approach_target = approach_target_.value_or(Point2{}),
      .preferred_z_m = preferred_z_m_,
      .vertical_error_m = vertical_error_m,
      .lateral_error_m = lateral_error_m,
      .approach_reference_speed_mps =
          std::min(config_.approach_alignment_speed_mps, input.passage_speed_limit_mps),
      .distance_to_entry_m = distance_to_entry_m,
      .capture_stable_cycles = capture_stable_cycles_,
      .retention_violation_cycles = retention_violation_cycles_,
      .required_alignment_time_s = alignment_time_s,
      .required_stopping_distance_m = stopping_distance_m,
      .required_alignment_distance_m = required_distance_m,
  };
  return result;
}

void PassageCoordinator::reset() noexcept {
  active_opening_.reset();
  travel_sign_ = 1.0;
  vertical_alignment_active_ = false;
  vertical_ready_latched_ = false;
  traversal_latched_ = false;
  partial_from_inside_ = false;
  capture_stable_cycles_ = 0U;
  retention_violation_cycles_ = 0U;
  hold_position_ = {};
  approach_target_.reset();
  preferred_z_m_ = 0.0;
}

const char* passageCoordinatorPhaseName(const PassageCoordinatorPhase phase) noexcept {
  switch (phase) {
    case PassageCoordinatorPhase::kInactive:
      return "inactive";
    case PassageCoordinatorPhase::kApproach:
      return "approach";
    case PassageCoordinatorPhase::kStationaryVerticalAlignment:
      return "stationary_vertical_alignment";
    case PassageCoordinatorPhase::kTraversal:
      return "traversal";
    case PassageCoordinatorPhase::kPartialFromInside:
      return "partial_from_inside";
    case PassageCoordinatorPhase::kInvalidOpening:
      return "invalid_opening";
  }
  return "unknown";
}

} // namespace drone_city_nav
