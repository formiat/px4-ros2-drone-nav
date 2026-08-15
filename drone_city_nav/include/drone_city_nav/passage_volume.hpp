#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace drone_city_nav {

struct PassageVolumeConfig {
  double cross_section_spacing_m{1.0};
  double lateral_probe_step_m{0.5};
  double secondary_probe_step_m{0.5};
  double maximum_cross_section_probe_m{30.0};
  double minimum_wall_clearance_m{1.0};
  FlightEnvelopeConfig flight_envelope{};
  SweptFootprintConfig footprint{};
};

struct PassageCrossSection {
  double station_m{0.0};
  Point3 center{};
  Vec3 tangent{};
  Vec3 lateral_axis{};
  Vec3 secondary_axis{};
  // Full raw-validated center-position range. Cooperative wall clearance is
  // intentionally not applied here because these bounds become the physical
  // execution envelope for the selected route.
  double minimum_lateral_offset_m{0.0};
  double maximum_lateral_offset_m{0.0};
  double minimum_secondary_offset_m{0.0};
  double maximum_secondary_offset_m{0.0};
  bool raw_validated{false};

  [[nodiscard]] double usableWidthM() const noexcept {
    return maximum_lateral_offset_m - minimum_lateral_offset_m;
  }

  [[nodiscard]] double usableSecondaryExtentM() const noexcept {
    return maximum_secondary_offset_m - minimum_secondary_offset_m;
  }
};

struct PassageVolume {
  PassageTraversalId passage_traversal_id;
  std::size_t span_index{0U};
  double begin_station_m{0.0};
  double end_station_m{0.0};
  // Intersection of cross-section ranges after applying the requested wall
  // clearance. These bounds constrain cooperative offset selection only.
  double minimum_lateral_offset_m{0.0};
  double maximum_lateral_offset_m{0.0};
  double minimum_secondary_offset_m{0.0};
  double maximum_secondary_offset_m{0.0};
  double minimum_physical_width_m{0.0};
  double minimum_physical_secondary_extent_m{0.0};
  std::vector<PassageCrossSection> cross_sections;
  bool raw_validated{false};
  std::vector<PassageTraversalSegmentSpan> segment_spans;

  [[nodiscard]] double usableWidthM() const noexcept {
    return maximum_lateral_offset_m - minimum_lateral_offset_m;
  }

  [[nodiscard]] bool exclusive(const double required_separation_m) const noexcept {
    return usableWidthM() + 1.0e-9 < required_separation_m;
  }
};

struct PassageVolumeResource {
  std::shared_ptr<const std::vector<PassageVolume>> volumes;
  bool shared_resource_reused{false};
};

[[nodiscard]] bool
passageVolumeConfigIsValid(const PassageVolumeConfig& config) noexcept;

[[nodiscard]] std::vector<PassageVolume>
derivePassageVolumes(std::span<const RouteSample3D> route,
                     std::span<const ConstrainedRouteSpan> constrained_spans,
                     const OccupancyGrid3D& occupancy,
                     const PassageVolumeConfig& config);

[[nodiscard]] PassageVolumeResource
acquireDerivedPassageVolumes(std::span<const RouteSample3D> route,
                             std::span<const ConstrainedRouteSpan> constrained_spans,
                             const OccupancyGrid3D& occupancy,
                             const PassageVolumeConfig& config);

[[nodiscard]] std::size_t
projectPassageVolumeEnvelopes(std::span<ConstrainedRouteSpan> constrained_spans,
                              std::span<const PassageVolume> passage_volumes,
                              const SweptFootprintConfig& footprint) noexcept;

[[nodiscard]] const PassageCrossSection*
nearestPassageCrossSection(const PassageVolume& volume, double station_m) noexcept;

} // namespace drone_city_nav
