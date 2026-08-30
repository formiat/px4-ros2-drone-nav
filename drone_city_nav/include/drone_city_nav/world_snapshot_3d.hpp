#pragma once

#include "drone_city_nav/mppi/mppi_types.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/portal_graph.hpp"
#include "drone_city_nav/world_generation.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace drone_city_nav {

class VersionedObservedRawWorld3D;

// One immutable, causally coherent local-world publication. Route, search,
// activation, and telemetry state deliberately do not belong here. Derived
// ESDF and topology resources are caches over the exact raw world identity and
// carry no independent hard-collision authority.
struct WorldSnapshot3D {
  LocalWorldGeneration local_world_generation{};
  std::uint64_t producer_instance_id{0U};
  std::uint64_t revision{0U};
  std::uint64_t source_raw_revision{0U};
  std::uint64_t source_occupied_fingerprint{0U};
  std::uint64_t raw_occupied_fingerprint{0U};
  std::uint64_t planner_parent_raw_revision{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t ready_stamp_ns{0};
  mppi::EsdfGrid grid{};
  std::shared_ptr<const std::vector<float>> distances_m;
  std::shared_ptr<const ObservedOccupancyGrid3D> observed_occupancy;
  std::shared_ptr<const OccupancyGrid3D> static_occupancy;
  std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world_owner;
  ObservedEsdfResource3D observed_esdf_resource{};
  std::vector<OccupancyChunkIndex3D> planner_dirty_chunks;
  std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed;
  std::optional<LaunchSupportContact3D> launch_support_contact;
  std::shared_ptr<const std::vector<PassageTraversalEdge>> topology_passage_traversals;
  bool launch_support_resolution_pending{false};
  bool planner_full_reset{false};
};

} // namespace drone_city_nav
