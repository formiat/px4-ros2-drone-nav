#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/raw_occupancy_clearance_3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "persistent_dstar_lite_planner_3d_internal.hpp"

// Soft clearance ranking of the lattice: node clearances cached lazily against
// per-chunk change stamps, the ranking factor curves, and the chunk flags that
// let open air rank at unity without a query.

namespace drone_city_nav::detail {

double PlannerLattice3D::rankedEdgeCost(const PersistentPlannerNode3D first,
                                        const PersistentPlannerNode3D second) {
  const double raw_cost = rawEdgeCost(first, second);
  if (!std::isfinite(raw_cost) || config_->clearance_ranking_weight <= 0.0) {
    return raw_cost;
  }
  // Soft ranking only: a traversable edge stays traversable, it just costs
  // more the closer its endpoints run to confirmed occupied evidence. The raw
  // flight time is cached separately from the clearance, so an occupied
  // change inside the ranking reach re-derives only the cheap node clearance
  // and never the swept-footprint validation.
  // The ranking clearance is measured from the body surface, as the execution
  // risk model measures it, so a node centre one body radius from a wall
  // ranks as touching rather than as one metre clear.
  const double reach_m = config_->clearance_ranking_distance_m;
  if (!nearOccupied(pointFor(first), reach_m) &&
      !nearOccupied(pointFor(second), reach_m)) {
    return raw_cost;
  }
  const double clearance_m =
      std::max(0.0, std::min(nodeClearanceM(first), nodeClearanceM(second)) -
                        config_->physical_footprint.radius_m);
  return raw_cost * rankingFactorForBodyClearance(clearance_m);
}

double PlannerLattice3D::rankedEdgeCost(const PersistentPlannerNode3D first,
                                        const PersistentPlannerNode3D second,
                                        const double clearance_reach_m) {
  const double raw_cost = rawEdgeCost(first, second);
  if (!std::isfinite(raw_cost) || config_->clearance_ranking_weight <= 0.0 ||
      !(clearance_reach_m > 0.0)) {
    return raw_cost;
  }
  // The curve is scaled to the reach: an edge clear beyond it costs its raw
  // flight time, so the unranked heuristic stays tight in open space and the
  // search remains directed; only the band near occupied evidence is priced.
  if (!nearOccupied(pointFor(first), clearance_reach_m) &&
      !nearOccupied(pointFor(second), clearance_reach_m)) {
    return raw_cost;
  }
  const double clearance_m =
      std::max(0.0, std::min(nodeClearanceWithin(first, clearance_reach_m),
                             nodeClearanceWithin(second, clearance_reach_m)) -
                        config_->physical_footprint.radius_m);
  return raw_cost * rankingFactorForBodyClearance(clearance_m, clearance_reach_m);
}

double PlannerLattice3D::rankingFactorForBodyClearance(
    const double body_clearance_m) const noexcept {
  return rankingFactorForBodyClearance(body_clearance_m,
                                       config_->clearance_ranking_distance_m);
}

double PlannerLattice3D::rankingFactorForBodyClearance(
    const double body_clearance_m, const double distance_m) const noexcept {
  if (!(config_->clearance_ranking_weight > 0.0) || !(distance_m > 0.0) ||
      body_clearance_m >= distance_m) {
    return 1.0;
  }
  const double shortfall = 1.0 - body_clearance_m / distance_m;
  double factor = 1.0 + config_->clearance_ranking_weight * shortfall * shortfall;
  const double critical_m = config_->clearance_ranking_critical_distance_m;
  if (config_->clearance_ranking_critical_weight > 0.0 && critical_m > 0.0 &&
      body_clearance_m < critical_m) {
    const double critical_shortfall = 1.0 - body_clearance_m / critical_m;
    factor += config_->clearance_ranking_critical_weight * critical_shortfall *
              critical_shortfall;
  }
  return factor;
}

double PlannerLattice3D::pointClearanceM(const Point3& point) const {
  const double cap_m = config_->clearance_ranking_distance_m;
  double clearance_m = cap_m;
  if (resident_collision_oracle_.has_value()) {
    const OccupiedCollisionWorld3D& world = resident_collision_oracle_->world();
    if (world.observed_occupancy != nullptr) {
      clearance_m = std::min(
          clearance_m, rawEuclideanClearance3D(*world.observed_occupancy, point, cap_m,
                                               world.launch_support_contact));
    }
    if (world.static_occupancy != nullptr) {
      clearance_m = std::min(
          clearance_m, rawEuclideanClearance3D(*world.static_occupancy, point, cap_m,
                                               world.launch_support_contact));
    }
  }
  return clearance_m;
}

double PlannerLattice3D::rankedSegmentTimeS(const Point3& first, const Point3& second,
                                            const double clearance_reach_m) const {
  const double segment_s =
      minimumFlightTranslationTime3D(first, second, config_->time_model);
  if (!std::isfinite(segment_s) || config_->clearance_ranking_weight <= 0.0 ||
      !(clearance_reach_m > 0.0)) {
    return segment_s;
  }
  const double radius_m = config_->physical_footprint.radius_m;
  const double sample_step_m = std::max(0.5, 0.5 * config_->minimum_horizontal_step_m);
  const double length_m = distance3D(first, second);
  const auto samples = static_cast<std::size_t>(std::ceil(length_m / sample_step_m));
  double worst_factor = 1.0;
  for (std::size_t sample = 0U; sample <= samples; ++sample) {
    const double ratio =
        samples == 0U ? 0.0
                      : static_cast<double>(sample) / static_cast<double>(samples);
    const Point3 point{first.x + ratio * (second.x - first.x),
                       first.y + ratio * (second.y - first.y),
                       first.z + ratio * (second.z - first.z)};
    if (!nearOccupied(point, clearance_reach_m)) {
      continue;
    }
    const double body_clearance_m =
        std::max(0.0, deriveNodeClearance(point, clearance_reach_m) - radius_m);
    worst_factor =
        std::max(worst_factor,
                 rankingFactorForBodyClearance(body_clearance_m, clearance_reach_m));
  }
  return segment_s * worst_factor;
}

double PlannerLattice3D::rankedPathTimeS(const std::vector<Point3>& path,
                                         const FlightPathTimeProfile3D& profile) const {
  if (path.size() < 2U || !profile.valid ||
      profile.arrival_times_s.size() != path.size() ||
      profile.departure_times_s.size() != path.size()) {
    return 0.0;
  }
  const double radius_m = config_->physical_footprint.radius_m;
  const double sample_step_m = std::max(0.5, 0.5 * config_->minimum_horizontal_step_m);
  double ranked_s = 0.0;
  for (std::size_t index = 0U; index + 1U < path.size(); ++index) {
    const Point3& first = path[index];
    const Point3& second = path[index + 1U];
    const double segment_s = std::max(0.0, profile.arrival_times_s[index + 1U] -
                                               profile.departure_times_s[index]);
    const double turn_s = std::max(0.0, profile.departure_times_s[index] -
                                            profile.arrival_times_s[index]);
    const double length_m = distance3D(first, second);
    const auto samples = static_cast<std::size_t>(std::ceil(length_m / sample_step_m));
    double worst_factor = 1.0;
    for (std::size_t sample = 0U; sample <= samples; ++sample) {
      const double ratio =
          samples == 0U ? 0.0
                        : static_cast<double>(sample) / static_cast<double>(samples);
      const Point3 point{first.x + ratio * (second.x - first.x),
                         first.y + ratio * (second.y - first.y),
                         first.z + ratio * (second.z - first.z)};
      const double body_clearance_m = std::max(0.0, pointClearanceM(point) - radius_m);
      worst_factor =
          std::max(worst_factor, rankingFactorForBodyClearance(body_clearance_m));
    }
    ranked_s += turn_s + segment_s * worst_factor;
  }
  return ranked_s;
}

void PlannerLattice3D::resetNodeClearances() noexcept {
  node_clearance_cache_.clear();
  std::ranges::fill(chunk_change_epoch_, 0U);
  change_epoch_ = 0U;
  moved_clearances_.clear();
}

void PlannerLattice3D::noteChangedChunks(
    const std::unordered_set<OccupancyChunkIndex3D, OccupancyChunkIndex3DHash>&
        changed_chunks) {
  if (changed_chunks.empty() || clearanceRankingReachM() <= 0.0) {
    return;
  }
  ++change_epoch_;
  for (const OccupancyChunkIndex3D& chunk : changed_chunks) {
    const std::optional<std::size_t> slot = chunkSlot(chunk);
    if (!slot.has_value()) {
      continue;
    }
    chunk_change_epoch_[*slot] = change_epoch_;
    markNearOccupied(chunk);
  }
}

std::optional<std::size_t>
PlannerLattice3D::chunkSlot(const OccupancyChunkIndex3D& chunk) const noexcept {
  if (chunk.x < 0 || chunk.y < 0 || chunk.z < 0 || chunk.x >= chunk_columns_ ||
      chunk.y >= chunk_rows_ || chunk.z >= chunk_layers_) {
    return std::nullopt;
  }
  return (static_cast<std::size_t>(chunk.z) * static_cast<std::size_t>(chunk_rows_) +
          static_cast<std::size_t>(chunk.y)) *
             static_cast<std::size_t>(chunk_columns_) +
         static_cast<std::size_t>(chunk.x);
}

void PlannerLattice3D::markNearOccupied(const OccupancyChunkIndex3D& chunk) noexcept {
  const int radius = near_occupied_chunk_radius_;
  for (int z = chunk.z - radius; z <= chunk.z + radius; ++z) {
    for (int y = chunk.y - radius; y <= chunk.y + radius; ++y) {
      for (int x = chunk.x - radius; x <= chunk.x + radius; ++x) {
        const std::optional<std::size_t> slot =
            chunkSlot(OccupancyChunkIndex3D{x, y, z});
        if (!slot.has_value()) {
          continue;
        }
        // Ring one is the chunk itself and its neighbours; the recorded ring
        // is the nearest one that ever held occupied evidence.
        const int ring = 1 + std::max({std::abs(x - chunk.x), std::abs(y - chunk.y),
                                       std::abs(z - chunk.z)});
        std::uint8_t& recorded = chunk_occupied_ring_[*slot];
        if (recorded == 0U || ring < recorded) {
          recorded = static_cast<std::uint8_t>(ring);
        }
      }
    }
  }
}

bool PlannerLattice3D::nearOccupied(const Point3& point,
                                    const double reach_m) const noexcept {
  if (chunk_occupied_ring_.empty()) {
    return true;
  }
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  const double chunk_span_m = kChunkSize * raw_bounds_.resolution_m;
  const OccupancyChunkIndex3D chunk{
      static_cast<int>(std::floor((point.x - raw_bounds_.origin_x) / chunk_span_m)),
      static_cast<int>(std::floor((point.y - raw_bounds_.origin_y) / chunk_span_m)),
      static_cast<int>(std::floor((point.z - raw_bounds_.origin_z) / chunk_span_m))};
  const std::optional<std::size_t> slot = chunkSlot(chunk);
  if (!slot.has_value()) {
    return true;
  }
  const std::uint8_t ring = chunk_occupied_ring_[*slot];
  if (ring == 0U) {
    return false;
  }
  // Evidence `ring` rings away lies at least (ring - 1) chunk spans away.
  const int rings_within_reach =
      1 + static_cast<int>(std::ceil(std::max(0.0, reach_m) / chunk_span_m));
  return ring <= rings_within_reach;
}

std::vector<PersistentPlannerNode3D> PlannerLattice3D::takeMovedClearances() {
  return std::exchange(moved_clearances_, {});
}

std::size_t PlannerLattice3D::clearancesRederived() const noexcept {
  return clearances_rederived_;
}

bool PlannerLattice3D::clearanceStale(const Point3& point, const double reach_m,
                                      const std::uint64_t change_epoch) const noexcept {
  if (change_epoch == change_epoch_ || chunk_change_epoch_.empty()) {
    return false;
  }
  constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  const double chunk_span_m = kChunkSize * raw_bounds_.resolution_m;
  const auto chunk_of = [&](const double coordinate, const double origin) {
    return static_cast<int>(std::floor((coordinate - origin) / chunk_span_m));
  };
  const int minimum_x = std::max(0, chunk_of(point.x - reach_m, raw_bounds_.origin_x));
  const int maximum_x =
      std::min(chunk_columns_ - 1, chunk_of(point.x + reach_m, raw_bounds_.origin_x));
  const int minimum_y = std::max(0, chunk_of(point.y - reach_m, raw_bounds_.origin_y));
  const int maximum_y =
      std::min(chunk_rows_ - 1, chunk_of(point.y + reach_m, raw_bounds_.origin_y));
  const int minimum_z = std::max(0, chunk_of(point.z - reach_m, raw_bounds_.origin_z));
  const int maximum_z =
      std::min(chunk_layers_ - 1, chunk_of(point.z + reach_m, raw_bounds_.origin_z));
  const double reach_squared = reach_m * reach_m;
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      const std::size_t row_offset =
          (static_cast<std::size_t>(z) * static_cast<std::size_t>(chunk_rows_) +
           static_cast<std::size_t>(y)) *
          static_cast<std::size_t>(chunk_columns_);
      for (int x = minimum_x; x <= maximum_x; ++x) {
        if (chunk_change_epoch_[row_offset + static_cast<std::size_t>(x)] <=
            change_epoch) {
          continue;
        }
        // The reach box over-approximates the reach sphere at the chunk
        // corners; a changed chunk whose box lies beyond the reach cannot
        // hold evidence within it.
        if (raw_occupancy_clearance_detail::chunkDistanceSquared3D(
                raw_bounds_, OccupancyChunkIndex3D{x, y, z}, point) <= reach_squared) {
          return true;
        }
      }
    }
  }
  return false;
}

std::optional<double>
PlannerLattice3D::cachedNodeClearance(const PersistentPlannerNode3D node,
                                      const double reach_m) {
  if (!node_clearance_cache_.contains(node)) {
    return std::nullopt;
  }
  return nodeClearanceWithin(node, reach_m);
}

double PlannerLattice3D::clearanceRankingReachM() const noexcept {
  return config_->clearance_ranking_weight > 0.0 ? config_->clearance_ranking_distance_m
                                                 : 0.0;
}

double PlannerLattice3D::deriveNodeClearance(const Point3& point,
                                             const double cap_m) const {
  double clearance_m = cap_m;
  if (resident_collision_oracle_.has_value()) {
    const OccupiedCollisionWorld3D& world = resident_collision_oracle_->world();
    if (world.observed_occupancy != nullptr) {
      clearance_m = std::min(
          clearance_m, rawEuclideanClearance3D(*world.observed_occupancy, point, cap_m,
                                               world.launch_support_contact));
    }
    if (world.static_occupancy != nullptr) {
      clearance_m = std::min(
          clearance_m, rawEuclideanClearance3D(*world.static_occupancy, point, cap_m,
                                               world.launch_support_contact));
    }
  }
  return clearance_m;
}

double PlannerLattice3D::nodeClearanceM(const PersistentPlannerNode3D node) {
  return nodeClearanceWithin(node, config_->clearance_ranking_distance_m);
}

double PlannerLattice3D::nodeClearanceWithin(const PersistentPlannerNode3D node,
                                             const double cap_m) {
  const Point3 point = pointFor(node);
  const auto found = node_clearance_cache_.find(node);
  if (found != node_clearance_cache_.end()) {
    CachedNodeClearance& cached = found->second;
    // A value derived within a shorter reach that reached its cap says only
    // that nothing lies closer than that cap; a longer reach re-derives it.
    const bool reach_sufficient =
        cached.cap_m + 1.0e-9 >= cap_m || cached.clearance_m + 1.0e-9 < cached.cap_m;
    // Only a change closer than the cached clearance can move it: occupied
    // evidence added farther away cannot lower a minimum, and evidence removed
    // farther away was not the nearest. A clearance that reached its cap
    // knows nothing beyond the cap, so any change within it still counts.
    const double relevant_reach_m = cached.clearance_m + 1.0e-9 < cached.cap_m
                                        ? cached.clearance_m + 1.0e-6
                                        : cached.cap_m;
    if (reach_sufficient &&
        !clearanceStale(point, relevant_reach_m, cached.change_epoch)) {
      cached.change_epoch = change_epoch_;
      return std::min(cached.clearance_m, cap_m);
    }
    // Occupied evidence within reach changed since this clearance was
    // derived, or a longer reach is asked for: re-derive it, and hand the
    // node to the search when the move matters for its ranked labels.
    const double previous_m = cached.clearance_m;
    const double reach_m = std::max(cached.cap_m, cap_m);
    cached.clearance_m = deriveNodeClearance(point, reach_m);
    cached.cap_m = reach_m;
    cached.change_epoch = change_epoch_;
    ++clearances_rederived_;
    if (rankingRepairRequired(previous_m, cached.clearance_m)) {
      moved_clearances_.push_back(node);
    }
    return std::min(cached.clearance_m, cap_m);
  }
  const double clearance_m = deriveNodeClearance(point, cap_m);
  node_clearance_cache_.emplace(node,
                                CachedNodeClearance{.clearance_m = clearance_m,
                                                    .cap_m = cap_m,
                                                    .change_epoch = change_epoch_});
  return clearance_m;
}

bool PlannerLattice3D::rankingRepairRequired(
    const double previous_clearance_m,
    const double current_clearance_m) const noexcept {
  constexpr double kRankingRepairRelativeTolerance{0.05};
  const double radius_m = config_->physical_footprint.radius_m;
  const double previous_factor =
      rankingFactorForBodyClearance(std::max(0.0, previous_clearance_m - radius_m));
  const double current_factor =
      rankingFactorForBodyClearance(std::max(0.0, current_clearance_m - radius_m));
  return std::abs(current_factor - previous_factor) >
         kRankingRepairRelativeTolerance * std::max(previous_factor, current_factor);
}

} // namespace drone_city_nav::detail
