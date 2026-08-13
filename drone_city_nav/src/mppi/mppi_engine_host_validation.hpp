#pragma once

[[nodiscard]] SweptFootprintConfig
hostFootprintConfig(const FootprintConfig& footprint) noexcept {
  return SweptFootprintConfig{
      .radius_m = footprint.radius_m,
      .lower_extent_m = footprint.lower_extent_m,
      .upper_extent_m = footprint.upper_extent_m,
      .perimeter_samples = footprint.perimeter_samples,
      .radial_rings = footprint.radial_rings,
      .axial_samples = footprint.axial_samples,
  };
}

[[nodiscard]] FootprintBodyAxis hostBodyAxis(const Control& control) noexcept {
  return bodyAxisFromWorldAcceleration(Vec3{control.ax, control.ay, control.az});
}

[[nodiscard]] bool hostSolidCollision(const State& state,
                                      const FootprintBodyAxis& body_axis,
                                      const FootprintConfig& footprint_config,
                                      const std::span<const KnownSolid> solids) {
  const SweptFootprintConfig footprint = hostFootprintConfig(footprint_config);
  const auto overlaps_projection = [&](const double center_projection,
                                       const double axis_projection,
                                       const double solid_min, const double solid_max) {
    const double projection = std::clamp(axis_projection, -1.0, 1.0);
    const double radial_projection =
        footprint.radius_m * std::sqrt(std::max(0.0, 1.0 - projection * projection));
    const double maximum_axial = projection >= 0.0
                                     ? footprint.upper_extent_m * projection
                                     : -footprint.lower_extent_m * projection;
    const double minimum_axial = projection >= 0.0
                                     ? -footprint.lower_extent_m * projection
                                     : footprint.upper_extent_m * projection;
    return center_projection + maximum_axial + radial_projection >= solid_min &&
           center_projection + minimum_axial - radial_projection <= solid_max;
  };
  return std::ranges::any_of(solids, [&](const KnownSolid& solid) {
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
    return overlaps_projection(
               depth, body_axis.x * solid.normal_x + body_axis.y * solid.normal_y,
               -solid.half_depth_m, solid.half_depth_m) &&
           overlaps_projection(
               lateral, body_axis.x * solid.lateral_x + body_axis.y * solid.lateral_y,
               -solid.half_width_m, solid.half_width_m) &&
           overlaps_projection(state.z, body_axis.z, solid.min_z_m, solid.max_z_m);
  });
}

[[nodiscard]] bool hostSweptSolidCollision(const std::span<const State> horizon,
                                           const std::span<const Control> controls,
                                           const FootprintConfig& footprint,
                                           const std::span<const KnownSolid> solids) {
  if (horizon.size() != controls.size() + 1U) {
    return true;
  }
  for (std::size_t index = 0U; index < controls.size(); ++index) {
    const Control& control = controls[index];
    const FootprintBodyAxis body_axis = hostBodyAxis(control);
    const State& previous = horizon[index];
    const State& next = horizon[index + 1U];
    const float segment_length_m = std::hypot(
        std::hypot(next.x - previous.x, next.y - previous.y), next.z - previous.z);
    const std::size_t samples = std::max<std::size_t>(
        1U, static_cast<std::size_t>(std::ceil(segment_length_m / 0.25F)));
    for (std::size_t sample = 1U; sample <= samples; ++sample) {
      const float ratio = static_cast<float>(sample) / static_cast<float>(samples);
      State state = next;
      state.x = std::lerp(previous.x, next.x, ratio);
      state.y = std::lerp(previous.y, next.y, ratio);
      state.z = std::lerp(previous.z, next.z, ratio);
      if (hostSolidCollision(state, body_axis, footprint, solids)) {
        return true;
      }
    }
  }
  return false;
}

struct EvaluatedControlSequence {
  ReferenceSimulationTrace trace;
  RolloutMetrics metrics{};
  MppiPostUpdateClassificationResult classification{};
  bool known_solid_collision{false};
};
