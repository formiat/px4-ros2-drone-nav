#include "drone_city_nav/clearance_field.hpp"

#include "drone_city_nav/distance_field.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] DistanceFieldSource
distanceFieldSourceForClearance(const ClearanceSource source) noexcept {
  switch (source) {
    case ClearanceSource::kOccupied:
      return DistanceFieldSource::kOccupied;
    case ClearanceSource::kProhibited:
      return DistanceFieldSource::kProhibited;
  }
  return DistanceFieldSource::kOccupied;
}

[[nodiscard]] std::size_t linearIndex(const GridBounds& bounds, const GridIndex cell) {
  if (cell.x < 0 || cell.y < 0 || cell.x >= bounds.width_cells ||
      cell.y >= bounds.height_cells) {
    throw std::out_of_range{"Clearance field cell is outside grid bounds"};
  }
  return static_cast<std::size_t>(cell.y) *
             static_cast<std::size_t>(bounds.width_cells) +
         static_cast<std::size_t>(cell.x);
}

[[nodiscard]] bool sameBounds(const GridBounds& lhs, const GridBounds& rhs) noexcept {
  return lhs.origin_x == rhs.origin_x && lhs.origin_y == rhs.origin_y &&
         lhs.resolution_m == rhs.resolution_m && lhs.width_cells == rhs.width_cells &&
         lhs.height_cells == rhs.height_cells;
}

} // namespace

ClearanceField2D ClearanceField2D::build(const OccupancyGrid2D& grid,
                                         const double max_distance_m,
                                         const ClearanceSource source) {
  ClearanceField2D field{};
  field.bounds_ = grid.bounds();
  field.max_distance_m_ = std::max(0.0, max_distance_m);
  field.source_ = source;
  field.distance_m_.assign(grid.cellCount(), std::numeric_limits<double>::infinity());
  if (field.max_distance_m_ <= 0.0) {
    return field;
  }

  const DistanceField2D distance_field = DistanceField2D::build(
      grid, field.max_distance_m_, distanceFieldSourceForClearance(source));
  const std::span<const double> distances = distance_field.distancesM();
  field.distance_m_.assign(distances.begin(), distances.end());
  return field;
}

const GridBounds& ClearanceField2D::bounds() const noexcept {
  return bounds_;
}

double ClearanceField2D::maxDistanceM() const noexcept {
  return max_distance_m_;
}

ClearanceSource ClearanceField2D::source() const noexcept {
  return source_;
}

bool ClearanceField2D::contains(const GridIndex cell) const noexcept {
  return cell.x >= 0 && cell.y >= 0 && cell.x < bounds_.width_cells &&
         cell.y < bounds_.height_cells;
}

double ClearanceField2D::distanceAt(const GridIndex cell) const {
  return distance_m_.at(linearIndex(bounds_, cell));
}

std::span<const double> ClearanceField2D::distancesM() const noexcept {
  return distance_m_;
}

ClearanceFieldCacheLookup
ClearanceFieldCache::getOrBuild(const OccupancyGrid2D& grid,
                                const double max_distance_m,
                                const ClearanceSource source) {
  if (matches(grid, max_distance_m, source) && field_.has_value()) {
    return ClearanceFieldCacheLookup{&*field_, true};
  }

  const OccupancyGridFingerprint fingerprint = grid.prohibitedFingerprint();
  bounds_ = fingerprint.bounds;
  max_distance_m_ = std::max(0.0, max_distance_m);
  source_ = source;
  cells_hash_ = fingerprint.cells_hash;
  inflated_hash_ = fingerprint.inflated_hash;
  field_ = ClearanceField2D::build(grid, max_distance_m_, source);
  return ClearanceFieldCacheLookup{&*field_, false};
}

void ClearanceFieldCache::clear() noexcept {
  bounds_ = GridBounds{};
  max_distance_m_ = 0.0;
  source_ = ClearanceSource::kOccupied;
  cells_hash_ = 0U;
  inflated_hash_ = 0U;
  field_.reset();
}

bool ClearanceFieldCache::matches(const OccupancyGrid2D& grid,
                                  const double max_distance_m,
                                  const ClearanceSource source) const noexcept {
  if (!field_.has_value() || source_ != source ||
      max_distance_m_ != std::max(0.0, max_distance_m) ||
      !sameBounds(bounds_, grid.bounds())) {
    return false;
  }

  const OccupancyGridFingerprint fingerprint = grid.prohibitedFingerprint();
  return cells_hash_ == fingerprint.cells_hash &&
         inflated_hash_ == fingerprint.inflated_hash;
}

} // namespace drone_city_nav
