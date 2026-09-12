#include "drone_city_nav/footprint_geometry_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finiteFootprintConfig(const SweptFootprintConfig& config) noexcept {
  return std::isfinite(config.radius_m) && config.radius_m >= 0.0 &&
         std::isfinite(config.body_radius_m) && config.body_radius_m >= 0.0 &&
         std::isfinite(config.body_lower_extent_m) &&
         config.body_lower_extent_m >= 0.0 &&
         std::isfinite(config.body_upper_extent_m) &&
         config.body_upper_extent_m >= 0.0 && std::isfinite(config.lower_extent_m) &&
         config.lower_extent_m >= 0.0 && std::isfinite(config.upper_extent_m) &&
         config.upper_extent_m >= 0.0 && std::isfinite(config.sweep_step_m) &&
         config.sweep_step_m > 0.0 &&
         std::isfinite(config.safe_clearance_threshold_m) &&
         config.safe_clearance_threshold_m >= 0.0 && config.axial_samples > 0U;
}

[[nodiscard]] bool finiteStrictBox(const AxisAlignedBox3D& box) noexcept {
  return std::isfinite(box.minimum.x) && std::isfinite(box.minimum.y) &&
         std::isfinite(box.minimum.z) && std::isfinite(box.maximum.x) &&
         std::isfinite(box.maximum.y) && std::isfinite(box.maximum.z) &&
         box.maximum.x > box.minimum.x && box.maximum.y > box.minimum.y &&
         box.maximum.z > box.minimum.z;
}

[[nodiscard]] bool
knownLaunchSupportEvidenceSource(const LaunchSupportEvidenceSource source) noexcept {
  switch (source) {
    case LaunchSupportEvidenceSource::kObservedOccupancy:
    case LaunchSupportEvidenceSource::kVehicleLandDetector:
      return true;
  }
  return false;
}

} // namespace

bool sameProprioceptiveFreeSpaceSeed3D(
    const ProprioceptiveFreeSpaceSeed3D& first,
    const ProprioceptiveFreeSpaceSeed3D& second) noexcept {
  return first.position.x == second.position.x &&
         first.position.y == second.position.y &&
         first.position.z == second.position.z &&
         first.body_axis.x == second.body_axis.x &&
         first.body_axis.y == second.body_axis.y &&
         first.body_axis.z == second.body_axis.z &&
         first.footprint.radius_m == second.footprint.radius_m &&
         first.footprint.body_radius_m == second.footprint.body_radius_m &&
         first.footprint.body_lower_extent_m == second.footprint.body_lower_extent_m &&
         first.footprint.body_upper_extent_m == second.footprint.body_upper_extent_m &&
         first.footprint.lower_extent_m == second.footprint.lower_extent_m &&
         first.footprint.upper_extent_m == second.footprint.upper_extent_m &&
         first.footprint.perimeter_samples == second.footprint.perimeter_samples &&
         first.footprint.radial_rings == second.footprint.radial_rings &&
         first.footprint.axial_samples == second.footprint.axial_samples &&
         first.footprint.sweep_step_m == second.footprint.sweep_step_m &&
         first.footprint.safe_clearance_threshold_m ==
             second.footprint.safe_clearance_threshold_m &&
         first.contact_tolerance_m == second.contact_tolerance_m &&
         first.departure_chain.size() == second.departure_chain.size() &&
         std::equal(first.departure_chain.begin(), first.departure_chain.end(),
                    second.departure_chain.begin(),
                    [](const Point3& left, const Point3& right) noexcept {
                      return left.x == right.x && left.y == right.y &&
                             left.z == right.z;
                    });
}

bool sameLaunchSupportContact3D(const LaunchSupportContact3D& first,
                                const LaunchSupportContact3D& second) noexcept {
  const auto same_box = [](const AxisAlignedBox3D& first_box,
                           const AxisAlignedBox3D& second_box) noexcept {
    return first_box.minimum.x == second_box.minimum.x &&
           first_box.minimum.y == second_box.minimum.y &&
           first_box.minimum.z == second_box.minimum.z &&
           first_box.maximum.x == second_box.maximum.x &&
           first_box.maximum.y == second_box.maximum.y &&
           first_box.maximum.z == second_box.maximum.z;
  };
  return sameProprioceptiveFreeSpaceSeed3D(first.seed, second.seed) &&
         first.contact_cells.size() == second.contact_cells.size() &&
         std::ranges::equal(first.contact_cells, second.contact_cells, same_box) &&
         first.occupied_evidence_cells == second.occupied_evidence_cells &&
         first.evidence_source == second.evidence_source &&
         first.maximum_lateral_departure_m == second.maximum_lateral_departure_m &&
         first.minimum_axial_departure_m == second.minimum_axial_departure_m &&
         first.maximum_axial_settling_m == second.maximum_axial_settling_m;
}

bool launchSupportContactValid3D(const LaunchSupportContact3D& contact) noexcept {
  const double axis_norm =
      std::hypot(std::hypot(contact.seed.body_axis.x, contact.seed.body_axis.y),
                 contact.seed.body_axis.z);
  constexpr double kNormalizedAxisTolerance{1.0e-6};
  if (!std::isfinite(contact.seed.position.x) ||
      !std::isfinite(contact.seed.position.y) ||
      !std::isfinite(contact.seed.position.z) || !std::isfinite(axis_norm) ||
      std::abs(axis_norm - 1.0) > kNormalizedAxisTolerance ||
      !finiteFootprintConfig(contact.seed.footprint) || contact.contact_cells.empty() ||
      !knownLaunchSupportEvidenceSource(contact.evidence_source) ||
      !std::isfinite(contact.maximum_lateral_departure_m) ||
      contact.maximum_lateral_departure_m < 0.0 ||
      !std::isfinite(contact.minimum_axial_departure_m) ||
      !std::isfinite(contact.maximum_axial_settling_m) ||
      contact.maximum_axial_settling_m < 0.0 ||
      contact.minimum_axial_departure_m > 0.0 ||
      contact.minimum_axial_departure_m < -contact.maximum_axial_settling_m ||
      contact.occupied_evidence_cells > contact.contact_cells.size()) {
    return false;
  }
  if ((contact.evidence_source == LaunchSupportEvidenceSource::kObservedOccupancy &&
       contact.occupied_evidence_cells == 0U) ||
      (contact.evidence_source == LaunchSupportEvidenceSource::kVehicleLandDetector &&
       contact.occupied_evidence_cells != 0U)) {
    return false;
  }
  for (std::size_t index = 0U; index < contact.contact_cells.size(); ++index) {
    if (!finiteStrictBox(contact.contact_cells[index])) {
      return false;
    }
    for (std::size_t previous = 0U; previous < index; ++previous) {
      const AxisAlignedBox3D& first = contact.contact_cells[previous];
      const AxisAlignedBox3D& second = contact.contact_cells[index];
      if (first.minimum.x == second.minimum.x && first.minimum.y == second.minimum.y &&
          first.minimum.z == second.minimum.z && first.maximum.x == second.maximum.x &&
          first.maximum.y == second.maximum.y && first.maximum.z == second.maximum.z) {
        return false;
      }
    }
  }
  return true;
}

bool updateLaunchSupportSettling(LaunchSupportContact3D& contact,
                                 const Point3& observed_position) noexcept {
  if (!launchSupportContactValid3D(contact) || !std::isfinite(observed_position.x) ||
      !std::isfinite(observed_position.y) || !std::isfinite(observed_position.z)) {
    return false;
  }
  const FootprintBodyAxis seed_axis =
      normalizedFootprintBodyAxis(contact.seed.body_axis);
  const Point3 delta{observed_position.x - contact.seed.position.x,
                     observed_position.y - contact.seed.position.y,
                     observed_position.z - contact.seed.position.z};
  const double axial =
      delta.x * seed_axis.x + delta.y * seed_axis.y + delta.z * seed_axis.z;
  constexpr double kSettlingToleranceM{1.0e-9};
  if (axial >= contact.minimum_axial_departure_m - kSettlingToleranceM ||
      axial < -contact.maximum_axial_settling_m - kSettlingToleranceM) {
    return false;
  }
  contact.minimum_axial_departure_m = axial;
  return true;
}

bool launchSupportEnvelopeContains3D(const LaunchSupportContact3D& contact,
                                     const Point3& candidate_position) noexcept {
  const FootprintBodyAxis seed_axis =
      normalizedFootprintBodyAxis(contact.seed.body_axis);
  const Point3 delta{candidate_position.x - contact.seed.position.x,
                     candidate_position.y - contact.seed.position.y,
                     candidate_position.z - contact.seed.position.z};
  const double axial =
      delta.x * seed_axis.x + delta.y * seed_axis.y + delta.z * seed_axis.z;
  const double lateral_squared = std::max(0.0, delta.x * delta.x + delta.y * delta.y +
                                                   delta.z * delta.z - axial * axial);
  constexpr double kDepartureToleranceM{1.0e-9};
  return axial >= contact.minimum_axial_departure_m - kDepartureToleranceM &&
         lateral_squared <=
             contact.maximum_lateral_departure_m * contact.maximum_lateral_departure_m +
                 kDepartureToleranceM;
}

bool launchSupportContactContainsCell3D(const LaunchSupportContact3D& contact,
                                        const Point3& box_minimum,
                                        const Point3& box_maximum) noexcept {
  constexpr double kCellAlignmentToleranceM{1.0e-6};
  return std::ranges::any_of(contact.contact_cells, [&](const AxisAlignedBox3D& box) {
    return std::abs(box.minimum.x - box_minimum.x) <= kCellAlignmentToleranceM &&
           std::abs(box.minimum.y - box_minimum.y) <= kCellAlignmentToleranceM &&
           std::abs(box.minimum.z - box_minimum.z) <= kCellAlignmentToleranceM &&
           std::abs(box.maximum.x - box_maximum.x) <= kCellAlignmentToleranceM &&
           std::abs(box.maximum.y - box_maximum.y) <= kCellAlignmentToleranceM &&
           std::abs(box.maximum.z - box_maximum.z) <= kCellAlignmentToleranceM;
  });
}

} // namespace drone_city_nav
