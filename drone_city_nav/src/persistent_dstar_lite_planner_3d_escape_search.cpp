#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

namespace drone_city_nav::detail {

EscapeSearch3D::EscapeSearch3D(const PersistentPlannerConfig3D& config,
                               PlannerLattice3D& lattice) noexcept
    : config_{std::addressof(config)},
      lattice_{std::addressof(lattice)} {
}

void EscapeSearch3D::reset() noexcept {
  initialized_ = false;
  exhausted_ = false;
  origin_ = {};
  origin_cell_ = 0U;
  span_ = 0;
  vertical_span_ = 0;
  side_ = 0;
  horizontal_step_m_ = 0.0;
  vertical_step_m_ = 0.0;
  settled_.clear();
  cost_m_.clear();
  parent_.clear();
  open_ = {};
  node_valid_.clear();
  explored_ = 0U;
  probes_ = 0U;
}

bool EscapeSearch3D::initialized() const noexcept {
  return initialized_;
}

bool EscapeSearch3D::exhausted() const noexcept {
  return exhausted_;
}

std::size_t EscapeSearch3D::exploredCells() const noexcept {
  return explored_;
}

std::size_t EscapeSearch3D::totalProbes() const noexcept {
  return probes_;
}

void EscapeSearch3D::begin(const Point3& start) {
  reset();
  const auto subdivisions = static_cast<int>(
      std::max<std::size_t>(config_->departure_refinement_subdivisions, 1U));
  const auto radius = static_cast<int>(config_->escape_search_radius_cells);
  horizontal_step_m_ =
      config_->minimum_horizontal_step_m / static_cast<double>(subdivisions);
  vertical_step_m_ =
      config_->minimum_vertical_step_m / static_cast<double>(subdivisions);
  span_ = radius * subdivisions;
  vertical_span_ = radius * subdivisions;
  side_ = 2 * span_ + 1;
  const std::size_t cells = static_cast<std::size_t>(side_) *
                            static_cast<std::size_t>(side_) *
                            static_cast<std::size_t>(2 * vertical_span_ + 1);
  settled_.assign(cells, 0U);
  cost_m_.assign(cells, std::numeric_limits<double>::infinity());
  parent_.assign(cells, kNoCell);
  origin_ = start;
  initialized_ = true;
  // The origin is the box centre by construction.
  origin_cell_ = static_cast<std::uint32_t>(
      (static_cast<std::size_t>(vertical_span_) * static_cast<std::size_t>(side_) +
       static_cast<std::size_t>(span_)) *
          static_cast<std::size_t>(side_) +
      static_cast<std::size_t>(span_));
  cost_m_[origin_cell_] = 0.0;
  open_.push(QueueEntry3D{.cost_m = 0.0, .cell = origin_cell_});
}

std::optional<std::uint32_t> EscapeSearch3D::cellAt(const int x, const int y,
                                                    const int z) const noexcept {
  if (x < -span_ || x > span_ || y < -span_ || y > span_ || z < -vertical_span_ ||
      z > vertical_span_) {
    return std::nullopt;
  }
  const std::size_t index =
      (static_cast<std::size_t>(z + vertical_span_) * static_cast<std::size_t>(side_) +
       static_cast<std::size_t>(y + span_)) *
          static_cast<std::size_t>(side_) +
      static_cast<std::size_t>(x + span_);
  return static_cast<std::uint32_t>(index);
}

EscapeSearch3D::CellOffset3D
EscapeSearch3D::offsetOf(const std::uint32_t cell) const noexcept {
  const auto side = static_cast<std::uint32_t>(side_);
  const std::uint32_t x = cell % side;
  const std::uint32_t y = (cell / side) % side;
  const std::uint32_t z = cell / (side * side);
  return CellOffset3D{static_cast<int>(x) - span_, static_cast<int>(y) - span_,
                      static_cast<int>(z) - vertical_span_};
}

Point3 EscapeSearch3D::pointOf(const std::uint32_t cell) const noexcept {
  const CellOffset3D offset = offsetOf(cell);
  return Point3{origin_.x + static_cast<double>(offset.x) * horizontal_step_m_,
                origin_.y + static_cast<double>(offset.y) * horizontal_step_m_,
                origin_.z + static_cast<double>(offset.z) * vertical_step_m_};
}

bool EscapeSearch3D::insideMap(const Point3& point) const noexcept {
  const GridBounds3D& bounds = lattice_->bounds();
  return point.x >= bounds.origin_x &&
         point.x < bounds.origin_x +
                       static_cast<double>(bounds.width_cells) * bounds.resolution_m &&
         point.y >= bounds.origin_y &&
         point.y < bounds.origin_y +
                       static_cast<double>(bounds.height_cells) * bounds.resolution_m &&
         point.z >= bounds.origin_z &&
         point.z < bounds.origin_z +
                       static_cast<double>(bounds.depth_cells) * bounds.resolution_m;
}

std::optional<PersistentPlannerNode3D>
EscapeSearch3D::exitFrom(const Point3& point, const bool from_origin,
                         const std::function<bool(PersistentPlannerNode3D)>& closed,
                         std::size_t& probes) {
  // Only nodes one lattice step around the point: the exit segment is short
  // by construction, and the flood fill brings the frontier to every node it
  // can reach anyway.
  const PersistentPlannerNode3D center = lattice_->nearestNode(point);
  for (int z_offset = -1; z_offset <= 1; ++z_offset) {
    for (int y_offset = -1; y_offset <= 1; ++y_offset) {
      for (int x_offset = -1; x_offset <= 1; ++x_offset) {
        const PersistentPlannerNode3D node{center.x + x_offset, center.y + y_offset,
                                           center.z + z_offset};
        if (!lattice_->nodeInside(node) || closed(node) ||
            !lattice_->pointInsideFlightEnvelope(lattice_->pointFor(node))) {
          continue;
        }
        auto validity = node_valid_.find(node);
        if (validity == node_valid_.end()) {
          ++probes;
          ++probes_;
          validity = node_valid_.emplace(node, lattice_->nodeValid(node)).first;
        }
        if (!validity->second) {
          continue;
        }
        ++probes;
        ++probes_;
        const Point3 target = lattice_->pointFor(node);
        const bool reachable = from_origin
                                   ? lattice_->departureSegmentValid(point, target)
                                   : lattice_->rawSegmentValid(point, target);
        if (reachable) {
          return node;
        }
      }
    }
  }
  return std::nullopt;
}

std::optional<EscapeSearch3D::Result3D>
EscapeSearch3D::advance(const Point3& start,
                        const std::function<bool(PersistentPlannerNode3D)>& closed,
                        const std::chrono::steady_clock::time_point deadline,
                        const std::size_t maximum_probes, std::size_t& probes) {
  probes = 0U;
  if (config_->escape_search_radius_cells == 0U) {
    return std::nullopt;
  }
  // The grid is anchored on the vehicle; a vehicle that moved by more than
  // one fine step is searched from where it stands now.
  if (!initialized_ || distance3D(start, origin_) > horizontal_step_m_) {
    begin(start);
  }
  const std::uint32_t origin_cell = origin_cell_;
  while (!open_.empty() && probes < maximum_probes &&
         std::chrono::steady_clock::now() < deadline) {
    const QueueEntry3D current = open_.top();
    open_.pop();
    if (settled_[current.cell] != 0U || current.cost_m > cost_m_[current.cell]) {
      continue;
    }
    settled_[current.cell] = 1U;
    ++explored_;
    const Point3 point = pointOf(current.cell);
    const bool from_origin = current.cell == origin_cell;
    if (const std::optional<PersistentPlannerNode3D> exit =
            exitFrom(point, from_origin, closed, probes);
        exit.has_value()) {
      Result3D result{.anchor = *exit, .waypoints = {}};
      for (std::uint32_t cell = current.cell; cell != origin_cell;
           cell = parent_[cell]) {
        result.waypoints.push_back(pointOf(cell));
      }
      std::ranges::reverse(result.waypoints);
      return result;
    }
    const CellOffset3D offset = offsetOf(current.cell);
    for (int z_step = -1; z_step <= 1; ++z_step) {
      for (int y_step = -1; y_step <= 1; ++y_step) {
        for (int x_step = -1; x_step <= 1; ++x_step) {
          if (x_step == 0 && y_step == 0 && z_step == 0) {
            continue;
          }
          const std::optional<std::uint32_t> neighbor =
              cellAt(offset.x + x_step, offset.y + y_step, offset.z + z_step);
          if (!neighbor.has_value() || settled_[*neighbor] != 0U) {
            continue;
          }
          const Point3 target = pointOf(*neighbor);
          // Beyond the map everything is unknown and therefore free; a fill
          // that wandered out there would never end and could not end at a
          // node.
          if (!lattice_->pointInsideFlightEnvelope(target) || !insideMap(target)) {
            continue;
          }
          const double candidate_cost =
              cost_m_[current.cell] + distance3D(point, target);
          if (candidate_cost >= cost_m_[*neighbor]) {
            continue;
          }
          ++probes;
          ++probes_;
          const bool valid = from_origin
                                 ? lattice_->departureSegmentValid(point, target)
                                 : lattice_->rawSegmentValid(point, target);
          if (!valid) {
            continue;
          }
          cost_m_[*neighbor] = candidate_cost;
          parent_[*neighbor] = current.cell;
          open_.push(QueueEntry3D{.cost_m = candidate_cost, .cell = *neighbor});
        }
      }
    }
  }
  exhausted_ = open_.empty();
  return std::nullopt;
}

} // namespace drone_city_nav::detail
