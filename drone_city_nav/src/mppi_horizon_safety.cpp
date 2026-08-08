#include "drone_city_nav/mppi_horizon_safety.hpp"

#include "drone_city_nav/swept_footprint.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <ranges>

namespace drone_city_nav {
namespace {

void constrainStateToFlightEnvelope(mppi::State& state,
                                    const FlightEnvelopeConfig& envelope) noexcept {
  if (state.z < static_cast<float>(envelope.minimum_target_z_m)) {
    state.z = static_cast<float>(envelope.minimum_target_z_m);
    state.vz = std::max(0.0F, state.vz);
  }
  const float maximum_target_z =
      static_cast<float>(envelope.maximum_target_z_m - 1.0e-3);
  if (state.z >= maximum_target_z) {
    state.z = maximum_target_z;
    state.vz = std::min(0.0F, state.vz);
  }
}

[[nodiscard]] bool
intersectsKnownSolid(const mppi::State& state, const FootprintBodyAxis& body_axis,
                     const SweptFootprintConfig& footprint,
                     const std::span<const mppi::KnownSolid> known_solids) noexcept {
  const auto overlaps_cylinder_projection =
      [&](const double center_projection, const double axis_projection,
          const double solid_min, const double solid_max) {
        const double clamped_axis_projection = std::clamp(axis_projection, -1.0, 1.0);
        const double radial_projection =
            footprint.radius_m *
            std::sqrt(
                std::max(0.0, 1.0 - clamped_axis_projection * clamped_axis_projection));
        const double maximum_axial =
            clamped_axis_projection >= 0.0
                ? footprint.upper_extent_m * clamped_axis_projection
                : -footprint.lower_extent_m * clamped_axis_projection;
        const double minimum_axial =
            clamped_axis_projection >= 0.0
                ? -footprint.lower_extent_m * clamped_axis_projection
                : footprint.upper_extent_m * clamped_axis_projection;
        const double body_min = center_projection + minimum_axial - radial_projection;
        const double body_max = center_projection + maximum_axial + radial_projection;
        return body_max >= solid_min && body_min <= solid_max;
      };
  return std::ranges::any_of(known_solids, [&](const mppi::KnownSolid& solid) {
    if (!(footprint.radius_m > 0.0)) {
      if (state.z < solid.min_z_m || state.z > solid.max_z_m) {
        return false;
      }
      const float dx = state.x - solid.center_x_m;
      const float dy = state.y - solid.center_y_m;
      return std::abs(dx * solid.normal_x + dy * solid.normal_y) <=
                 solid.half_depth_m &&
             std::abs(dx * solid.lateral_x + dy * solid.lateral_y) <=
                 solid.half_width_m;
    }
    const float dx = state.x - solid.center_x_m;
    const float dy = state.y - solid.center_y_m;
    const double depth = dx * solid.normal_x + dy * solid.normal_y;
    const double lateral = dx * solid.lateral_x + dy * solid.lateral_y;
    const double axis_depth =
        body_axis.x * solid.normal_x + body_axis.y * solid.normal_y;
    const double axis_lateral =
        body_axis.x * solid.lateral_x + body_axis.y * solid.lateral_y;
    return overlaps_cylinder_projection(depth, axis_depth, -solid.half_depth_m,
                                        solid.half_depth_m) &&
           overlaps_cylinder_projection(lateral, axis_lateral, -solid.half_width_m,
                                        solid.half_width_m) &&
           overlaps_cylinder_projection(state.z, body_axis.z, solid.min_z_m,
                                        solid.max_z_m);
  });
}

[[nodiscard]] FootprintBodyAxis bodyAxisForSegment(const mppi::State& first,
                                                   const mppi::State& second,
                                                   const double duration_s) noexcept {
  const double safe_duration_s = std::max(1.0e-3, duration_s);
  return bodyAxisFromWorldAcceleration(Vec3{(second.vx - first.vx) / safe_duration_s,
                                            (second.vy - first.vy) / safe_duration_s,
                                            (second.vz - first.vz) / safe_duration_s});
}

[[nodiscard]] bool
intersectsPhysicalFootprint(const Point3& point, const mppi::State& state,
                            const FootprintBodyAxis& body_axis,
                            const SweptFootprintConfig& footprint) noexcept {
  const Point3 delta{point.x - static_cast<double>(state.x),
                     point.y - static_cast<double>(state.y),
                     point.z - static_cast<double>(state.z)};
  const double axial =
      delta.x * body_axis.x + delta.y * body_axis.y + delta.z * body_axis.z;
  if (axial < -footprint.lower_extent_m || axial > footprint.upper_extent_m) {
    return false;
  }
  const double distance_squared =
      delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
  const double radial_squared = std::max(0.0, distance_squared - axial * axial);
  return radial_squared <= footprint.radius_m * footprint.radius_m;
}

struct LatestLidarStoppingPathValidation {
  bool collision{false};
  double time_to_collision_s{std::numeric_limits<double>::infinity()};
  std::size_t validation_samples{0U};
  std::size_t point_checks{0U};
};

[[nodiscard]] mppi::State interpolateState(const mppi::State& first,
                                           const mppi::State& second,
                                           double ratio) noexcept;

[[nodiscard]] mppi::Control brakingControl(const mppi::State& state,
                                           const MppiHorizonSafetyConfig& config,
                                           const double duration_s) noexcept {
  const double horizontal_speed = std::hypot(state.vx, state.vy);
  mppi::Control control{};
  if (horizontal_speed > 1.0e-3) {
    const double deceleration = std::min(config.maximum_braking_acceleration_mps2,
                                         horizontal_speed / duration_s);
    control.ax = static_cast<float>(-deceleration * state.vx / horizontal_speed);
    control.ay = static_cast<float>(-deceleration * state.vy / horizontal_speed);
  }
  if (std::abs(state.vz) > 1.0e-3) {
    control.az = static_cast<float>(
        -std::copysign(std::min(config.maximum_braking_acceleration_mps2,
                                std::abs(state.vz) / duration_s),
                       state.vz));
  }
  return control;
}

void advanceState(mppi::State& state, const mppi::Control& control,
                  const double duration_s,
                  const FlightEnvelopeConfig& flight_envelope) noexcept {
  const float duration = static_cast<float>(duration_s);
  state.vx += control.ax * duration;
  state.vy += control.ay * duration;
  state.vz += control.az * duration;
  state.x += state.vx * duration;
  state.y += state.vy * duration;
  state.z += state.vz * duration;
  constrainStateToFlightEnvelope(state, flight_envelope);
}

[[nodiscard]] bool
validateLatestLidarState(const mppi::State& state, const FootprintBodyAxis& body_axis,
                         const SweptFootprintConfig& footprint,
                         const std::span<const Point3> latest_lidar_hit_points_map_m,
                         LatestLidarStoppingPathValidation& validation) noexcept {
  ++validation.validation_samples;
  for (const Point3& hit : latest_lidar_hit_points_map_m) {
    ++validation.point_checks;
    if (intersectsPhysicalFootprint(hit, state, body_axis, footprint)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] LatestLidarStoppingPathValidation validateLatestLidarStoppingPath(
    const mppi::State& current_state, const MppiHorizonSafetyConfig& config,
    const SweptFootprintConfig& footprint,
    const std::span<const Point3> latest_lidar_hit_points_map_m) {
  LatestLidarStoppingPathValidation validation;
  if (latest_lidar_hit_points_map_m.empty()) {
    return validation;
  }

  mppi::State previous = current_state;
  constrainStateToFlightEnvelope(previous, config.flight_envelope);
  if (!validateLatestLidarState(previous, FootprintBodyAxis{}, footprint,
                                latest_lidar_hit_points_map_m, validation)) {
    validation.collision = true;
    validation.time_to_collision_s = 0.0;
    return validation;
  }

  const double integration_step_s = std::max(1.0e-3, config.dt_s);
  const double stopping_duration_s = std::max(
      config.fallback_duration_s,
      config.reaction_latency_s +
          std::hypot(std::hypot(current_state.vx, current_state.vy), current_state.vz) /
              std::max(1.0e-3, config.maximum_braking_acceleration_mps2));
  double elapsed_s = 0.0;
  while (elapsed_s + 1.0e-9 < stopping_duration_s) {
    const bool reaction_phase = elapsed_s + 1.0e-9 < config.reaction_latency_s;
    const double phase_remaining_s = reaction_phase
                                         ? config.reaction_latency_s - elapsed_s
                                         : stopping_duration_s - elapsed_s;
    const double duration_s = std::min(
        {integration_step_s, stopping_duration_s - elapsed_s, phase_remaining_s});
    const mppi::Control control =
        reaction_phase ? mppi::Control{} : brakingControl(previous, config, duration_s);
    mppi::State next = previous;
    advanceState(next, control, duration_s, config.flight_envelope);
    const FootprintBodyAxis body_axis = bodyAxisForSegment(previous, next, duration_s);
    const double segment_length_m =
        std::hypot(std::hypot(static_cast<double>(next.x - previous.x),
                              static_cast<double>(next.y - previous.y)),
                   static_cast<double>(next.z - previous.z));
    const std::size_t sample_count = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(
                segment_length_m / std::max(1.0e-3, config.swept_validation_step_m))));
    for (std::size_t sample = 1U; sample <= sample_count; ++sample) {
      const double ratio =
          static_cast<double>(sample) / static_cast<double>(sample_count);
      const mppi::State state = interpolateState(previous, next, ratio);
      if (!validateLatestLidarState(state, body_axis, footprint,
                                    latest_lidar_hit_points_map_m, validation)) {
        validation.collision = true;
        validation.time_to_collision_s = elapsed_s + ratio * duration_s;
        return validation;
      }
    }
    elapsed_s += duration_s;
    previous = next;
    const double speed = std::hypot(std::hypot(previous.vx, previous.vy), previous.vz);
    if (!reaction_phase && speed <= 1.0e-3) {
      break;
    }
  }
  return validation;
}

void populateBrakingFallback(const mppi::State& initial,
                             const MppiHorizonSafetyConfig& config,
                             MppiHorizonSafetyResult& result) {
  mppi::State state = initial;
  constrainStateToFlightEnvelope(state, config.flight_envelope);
  const std::size_t steps =
      static_cast<std::size_t>(std::ceil(config.fallback_duration_s / config.dt_s));
  result.fallback_horizon.reserve(steps + 1U);
  result.fallback_controls.reserve(steps);
  result.fallback_horizon.push_back(state);
  for (std::size_t step = 0U; step < steps; ++step) {
    const mppi::Control control = brakingControl(state, config, config.dt_s);
    advanceState(state, control, config.dt_s, config.flight_envelope);
    result.fallback_controls.push_back(control);
    result.fallback_horizon.push_back(state);
  }
}

[[nodiscard]] mppi::State interpolateState(const mppi::State& first,
                                           const mppi::State& second,
                                           const double ratio) noexcept {
  const float value = static_cast<float>(std::clamp(ratio, 0.0, 1.0));
  return mppi::State{
      .x = std::lerp(first.x, second.x, value),
      .y = std::lerp(first.y, second.y, value),
      .z = std::lerp(first.z, second.z, value),
      .vx = std::lerp(first.vx, second.vx, value),
      .vy = std::lerp(first.vy, second.vy, value),
      .vz = std::lerp(first.vz, second.vz, value),
      .yaw = std::lerp(first.yaw, second.yaw, value),
      .yaw_rate = std::lerp(first.yaw_rate, second.yaw_rate, value),
  };
}

} // namespace

MppiHorizonSafetyResult
buildMppiBrakingFallback(const mppi::State& current_state,
                         const MppiHorizonSafetyConfig& config) {
  MppiHorizonSafetyResult result;
  const double speed =
      std::hypot(std::hypot(current_state.vx, current_state.vy), current_state.vz);
  result.decision = speed > 0.15 ? MppiHorizonSafetyDecision::kBrake
                                 : MppiHorizonSafetyDecision::kHold;
  result.stopping_time_s =
      config.reaction_latency_s +
      speed / std::max(1.0e-3, config.maximum_braking_acceleration_mps2);
  result.stopping_distance_m =
      speed * config.reaction_latency_s +
      speed * speed /
          (2.0 * std::max(1.0e-3, config.maximum_braking_acceleration_mps2));
  result.time_to_collision_s = std::numeric_limits<double>::infinity();
  result.latest_safe_intervention_time_s = 0.0;
  populateBrakingFallback(current_state, config, result);
  return result;
}

MppiHorizonSafetyResult evaluateMppiHorizonSafety(
    const mppi::State& current_state, const std::span<const mppi::State> horizon,
    const std::span<const float> esdf_m, const mppi::EsdfGrid& grid,
    const MppiHorizonSafetyConfig& config, const bool engine_collision,
    const std::span<const mppi::KnownSolid> known_solids,
    const OccupancyGrid3D* global_raw_occupancy,
    const OccupancyGrid2D* latest_raw_occupancy,
    const std::span<const Point3> latest_lidar_hit_points_map_m) {
  MppiHorizonSafetyResult result;
  const double speed =
      std::hypot(std::hypot(current_state.vx, current_state.vy), current_state.vz);
  result.stopping_time_s =
      config.reaction_latency_s +
      speed / std::max(1.0e-3, config.maximum_braking_acceleration_mps2);
  result.stopping_distance_m =
      speed * config.reaction_latency_s +
      speed * speed /
          (2.0 * std::max(1.0e-3, config.maximum_braking_acceleration_mps2));
  result.time_to_collision_s = std::numeric_limits<double>::infinity();
  mppi::State previous = current_state;
  const SweptFootprintConfig footprint{
      .radius_m = config.physical_footprint_radius_m,
      .lower_extent_m = config.physical_footprint_lower_extent_m,
      .upper_extent_m = config.physical_footprint_upper_extent_m,
      .perimeter_samples = config.physical_footprint_samples,
      .radial_rings = config.physical_footprint_radial_rings,
      .axial_samples = config.physical_footprint_axial_samples,
      .sweep_step_m = config.swept_validation_step_m};
  const LatestLidarStoppingPathValidation stopping_validation =
      validateLatestLidarStoppingPath(current_state, config, footprint,
                                      latest_lidar_hit_points_map_m);
  result.latest_lidar_stopping_path_collision = stopping_validation.collision;
  result.latest_lidar_stopping_time_to_collision_s =
      stopping_validation.time_to_collision_s;
  result.latest_lidar_stopping_validation_samples =
      stopping_validation.validation_samples;
  result.latest_lidar_stopping_point_checks = stopping_validation.point_checks;
  if (stopping_validation.collision) {
    result.latest_lidar_collision = true;
    result.time_to_collision_s = stopping_validation.time_to_collision_s;
    result.latest_safe_intervention_time_s = 0.0;
    result.decision = result.time_to_collision_s > config.minimum_time_to_collision_s
                          ? MppiHorizonSafetyDecision::kBrake
                          : MppiHorizonSafetyDecision::kHold;
    populateBrakingFallback(current_state, config, result);
    return result;
  }
  for (std::size_t index = 0U; index < horizon.size(); ++index) {
    const mppi::State& next = horizon[index];
    const FootprintBodyAxis body_axis = bodyAxisForSegment(previous, next, config.dt_s);
    const double segment_length_m =
        std::hypot(std::hypot(static_cast<double>(next.x - previous.x),
                              static_cast<double>(next.y - previous.y)),
                   static_cast<double>(next.z - previous.z));
    const double validation_step_m = std::max(1.0e-3, config.swept_validation_step_m);
    const std::size_t sample_count = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(segment_length_m / validation_step_m)));
    for (std::size_t sample = 1U; sample <= sample_count; ++sample) {
      const double ratio =
          static_cast<double>(sample) / static_cast<double>(sample_count);
      const mppi::State state = interpolateState(previous, next, ratio);
      if (!insideFlightEnvelope(state.z, config.flight_envelope)) {
        result.time_to_collision_s =
            index == 0U ? 0.0 : (static_cast<double>(index - 1U) + ratio) * config.dt_s;
        result.flight_envelope_violation = true;
        break;
      }
      const SweptFootprintResult footprint_validation = validateFootprintAt(
          grid, esdf_m, Point3{state.x, state.y, state.z}, body_axis, footprint);
      const bool outside_local_esdf =
          footprint_validation.status == SweptFootprintStatus::kOutsideGrid;
      if (outside_local_esdf && global_raw_occupancy != nullptr) {
        ++result.global_raw_fallback_samples;
        ++result.global_raw_validation_samples;
      }
      const bool static_global_raw_collision =
          outside_local_esdf && global_raw_occupancy != nullptr &&
          !validateRawFootprintAt(*global_raw_occupancy,
                                  Point3{state.x, state.y, state.z}, body_axis,
                                  footprint)
               .accepted();
      if (latest_raw_occupancy != nullptr) {
        ++result.global_raw_validation_samples;
      }
      const bool latest_raw_collision =
          latest_raw_occupancy != nullptr &&
          !validateRawFootprintAt(*latest_raw_occupancy,
                                  Point3{state.x, state.y, state.z}, footprint)
               .accepted();
      const bool global_raw_collision =
          static_global_raw_collision || latest_raw_collision;
      result.global_raw_collision = result.global_raw_collision || global_raw_collision;
      bool latest_lidar_collision = false;
      if (!latest_lidar_hit_points_map_m.empty()) {
        ++result.latest_lidar_validation_samples;
        for (const Point3& hit : latest_lidar_hit_points_map_m) {
          ++result.latest_lidar_point_checks;
          if (intersectsPhysicalFootprint(hit, state, body_axis, footprint)) {
            latest_lidar_collision = true;
            break;
          }
        }
      }
      result.latest_lidar_collision =
          result.latest_lidar_collision || latest_lidar_collision;
      const bool footprint_collision =
          footprint_validation.status == SweptFootprintStatus::kRawCollision ||
          footprint_validation.status == SweptFootprintStatus::kInvalidEsdf ||
          global_raw_collision || latest_lidar_collision;
      if (footprint_collision ||
          intersectsKnownSolid(state, body_axis, footprint, known_solids)) {
        result.time_to_collision_s =
            index == 0U ? 0.0 : (static_cast<double>(index - 1U) + ratio) * config.dt_s;
        break;
      }
    }
    if (std::isfinite(result.time_to_collision_s)) {
      break;
    }
    previous = next;
  }
  if (!std::isfinite(result.time_to_collision_s) && !engine_collision) {
    result.decision = MppiHorizonSafetyDecision::kExecute;
    result.latest_safe_intervention_time_s = std::numeric_limits<double>::infinity();
    return result;
  }
  if (!std::isfinite(result.time_to_collision_s)) {
    return buildMppiBrakingFallback(current_state, config);
  }
  result.latest_safe_intervention_time_s =
      std::max(0.0, result.time_to_collision_s - result.stopping_time_s);
  if (result.latest_safe_intervention_time_s > 0.0) {
    result.decision = MppiHorizonSafetyDecision::kExecuteUntilDeadline;
    populateBrakingFallback(current_state, config, result);
    return result;
  }
  result.decision = result.time_to_collision_s > config.minimum_time_to_collision_s
                        ? MppiHorizonSafetyDecision::kBrake
                        : MppiHorizonSafetyDecision::kHold;
  populateBrakingFallback(current_state, config, result);
  return result;
}

MppiSafetyInterventionUpdate
MppiSafetyInterventionTracker::update(const std::int64_t now_ns,
                                      const MppiHorizonSafetyResult& result) noexcept {
  if (result.decision == MppiHorizonSafetyDecision::kExecute) {
    reset();
    return {.decision = MppiHorizonSafetyDecision::kExecute,
            .deadline_ns = std::nullopt};
  }
  if (result.decision == MppiHorizonSafetyDecision::kExecuteUntilDeadline) {
    const auto candidate_ns =
        now_ns +
        static_cast<std::int64_t>(result.latest_safe_intervention_time_s * 1.0e9);
    if (!deadline_ns_.has_value() || candidate_ns < *deadline_ns_) {
      deadline_ns_ = candidate_ns;
    }
    if (now_ns < *deadline_ns_) {
      return {.decision = MppiHorizonSafetyDecision::kExecuteUntilDeadline,
              .deadline_ns = deadline_ns_};
    }
    return {.decision = MppiHorizonSafetyDecision::kBrake, .deadline_ns = deadline_ns_};
  }
  deadline_ns_ = now_ns;
  return {.decision = result.decision, .deadline_ns = deadline_ns_};
}

void MppiSafetyInterventionTracker::reset() noexcept {
  deadline_ns_.reset();
}

MppiBrakeHoldUpdate
MppiBrakeHoldLifecycle::update(const bool braking_required,
                               const mppi::State& current_state,
                               const double capture_speed_mps,
                               const FlightEnvelopeConfig& flight_envelope) noexcept {
  if (!braking_required) {
    reset();
    return {};
  }
  if (hold_state_.has_value() &&
      !insideFlightEnvelope(static_cast<double>(hold_state_->z), flight_envelope) &&
      insideFlightEnvelope(static_cast<double>(current_state.z), flight_envelope)) {
    hold_state_.reset();
  }
  if (!hold_state_.has_value()) {
    const double speed = std::hypot(std::hypot(static_cast<double>(current_state.vx),
                                               static_cast<double>(current_state.vy)),
                                    static_cast<double>(current_state.vz));
    if (speed <= std::max(0.0, capture_speed_mps) &&
        insideFlightEnvelope(static_cast<double>(current_state.z), flight_envelope)) {
      hold_state_ = current_state;
      hold_state_->vx = 0.0F;
      hold_state_->vy = 0.0F;
      hold_state_->vz = 0.0F;
    }
  }
  return hold_state_.has_value()
             ? MppiBrakeHoldUpdate{.position_hold = true, .hold_state = *hold_state_}
             : MppiBrakeHoldUpdate{};
}

void MppiBrakeHoldLifecycle::reset() noexcept {
  hold_state_.reset();
}

} // namespace drone_city_nav
