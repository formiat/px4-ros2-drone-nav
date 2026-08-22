#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <ranges>
#include <tuple>
#include <unordered_map>

#include "incremental_topology_graph_3d_internal.hpp"

namespace drone_city_nav::incremental_topology_detail {
namespace {

[[nodiscard]] bool betterRepresentative(const ObservationFrontier& candidate,
                                        const ObservationFrontier& current) {
  return std::tie(candidate.information_gain_voxels, candidate.supporting_rays,
                  candidate.minimum_known_free_ray_m) >
             std::tie(current.information_gain_voxels, current.supporting_rays,
                      current.minimum_known_free_ray_m) ||
         (std::tie(candidate.information_gain_voxels, candidate.supporting_rays,
                   candidate.minimum_known_free_ray_m) ==
              std::tie(current.information_gain_voxels, current.supporting_rays,
                       current.minimum_known_free_ray_m) &&
          std::tie(candidate.observation_pose.x, candidate.observation_pose.y,
                   candidate.observation_pose.z) <
              std::tie(current.observation_pose.x, current.observation_pose.y,
                       current.observation_pose.z));
}

[[nodiscard]] auto cellOrder(const GridIndex3D cell) noexcept {
  return std::tuple{cell.z, cell.y, cell.x};
}

[[nodiscard]] double squaredCellDistance(const GridIndex3D first,
                                         const GridIndex3D second) noexcept {
  const double dx = static_cast<double>(first.x - second.x);
  const double dy = static_cast<double>(first.y - second.y);
  const double dz = static_cast<double>(first.z - second.z);
  return dx * dx + dy * dy + dz * dz;
}

[[nodiscard]] std::vector<GridIndex3D>
selectSpatialRepresentatives(std::vector<GridIndex3D> candidates,
                             const std::size_t maximum_evaluations) {
  std::ranges::sort(candidates, {}, cellOrder);
  if (candidates.size() <= maximum_evaluations) {
    return candidates;
  }

  const auto extrema = [&](const auto projection, const bool maximum) {
    return *std::ranges::min_element(
        candidates,
        [projection, maximum](const GridIndex3D first, const GridIndex3D second) {
          const int first_value = projection(first);
          const int second_value = projection(second);
          if (first_value != second_value) {
            return maximum ? first_value > second_value : first_value < second_value;
          }
          return cellOrder(first) < cellOrder(second);
        });
  };

  std::vector<GridIndex3D> selected;
  selected.reserve(maximum_evaluations);
  const auto append_unique = [&](const GridIndex3D cell) {
    if (selected.size() < maximum_evaluations &&
        std::ranges::find(selected, cell) == selected.end()) {
      selected.push_back(cell);
    }
  };
  using CellProjection = int (*)(GridIndex3D);
  const std::array<CellProjection, 3U> projections{
      +[](const GridIndex3D cell) { return cell.x; },
      +[](const GridIndex3D cell) { return cell.y; },
      +[](const GridIndex3D cell) { return cell.z; },
  };
  for (const CellProjection projection : projections) {
    append_unique(extrema(projection, false));
    append_unique(extrema(projection, true));
  }

  while (selected.size() < maximum_evaluations) {
    const GridIndex3D* best = nullptr;
    double best_distance = -1.0;
    for (const GridIndex3D candidate : candidates) {
      if (std::ranges::find(selected, candidate) != selected.end()) {
        continue;
      }
      double minimum_distance = std::numeric_limits<double>::infinity();
      for (const GridIndex3D representative : selected) {
        minimum_distance =
            std::min(minimum_distance, squaredCellDistance(candidate, representative));
      }
      if (minimum_distance > best_distance ||
          (minimum_distance == best_distance &&
           (best == nullptr || cellOrder(candidate) < cellOrder(*best)))) {
        best = &candidate;
        best_distance = minimum_distance;
      }
    }
    if (best == nullptr) {
      break;
    }
    selected.push_back(*best);
  }
  return selected;
}

} // namespace

std::vector<ObservationFrontier> selectComponentObservationFrontiers(
    const ObservedOccupancyGrid3D& occupancy, const std::span<const GridIndex3D> cells,
    const std::uint64_t update_revision, const SensorObservabilityConfig& observability,
    const std::size_t maximum_evaluations, const std::size_t maximum_frontiers) {
  std::vector<GridIndex3D> candidates;
  candidates.reserve(cells.size());
  std::ranges::copy_if(cells, std::back_inserter(candidates), [&](const auto cell) {
    return observationPoseHasSupportedUnknownBoundary(occupancy, cell, observability);
  });
  candidates = selectSpatialRepresentatives(std::move(candidates), maximum_evaluations);

  std::unordered_map<std::uint64_t, ObservationFrontier> frontier_by_id;
  for (std::size_t index = 0U; index < candidates.size(); ++index) {
    const ObservationFrontierSetEvaluation evaluation =
        evaluateObservationFrontiers(occupancy, occupancy.cellCenter(candidates[index]),
                                     update_revision, observability);
    for (const ObservationFrontier& frontier : evaluation.frontiers) {
      auto [found, inserted] = frontier_by_id.try_emplace(frontier.id.value, frontier);
      if (!inserted && betterRepresentative(frontier, found->second)) {
        found->second = frontier;
      }
    }
  }

  std::vector<ObservationFrontier> result;
  result.reserve(frontier_by_id.size());
  for (auto& [id, frontier] : frontier_by_id) {
    static_cast<void>(id);
    result.push_back(std::move(frontier));
  }
  std::ranges::sort(
      result, [](const ObservationFrontier& first, const ObservationFrontier& second) {
        if (betterRepresentative(first, second)) {
          return true;
        }
        if (betterRepresentative(second, first)) {
          return false;
        }
        return first.id < second.id;
      });
  if (result.size() > maximum_frontiers) {
    result.resize(maximum_frontiers);
  }
  return result;
}

} // namespace drone_city_nav::incremental_topology_detail
