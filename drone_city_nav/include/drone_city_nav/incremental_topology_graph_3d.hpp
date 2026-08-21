#pragma once

#include "drone_city_nav/observation_frontier.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <compare>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {

struct IncrementalTopologyNodeId {
  std::uint64_t value{0U};

  [[nodiscard]] auto
  operator<=>(const IncrementalTopologyNodeId&) const noexcept = default;
};

struct IncrementalTopologyEdgeId {
  std::uint64_t value{0U};

  [[nodiscard]] auto
  operator<=>(const IncrementalTopologyEdgeId&) const noexcept = default;
};

struct IncrementalTopologyTileIndex3D {
  int x{0};
  int y{0};
  int z{0};

  [[nodiscard]] auto
  operator<=>(const IncrementalTopologyTileIndex3D&) const noexcept = default;
};

struct IncrementalTopologyNodeIdHash {
  [[nodiscard]] std::size_t operator()(IncrementalTopologyNodeId id) const noexcept;
};

struct IncrementalTopologyEdgeIdHash {
  [[nodiscard]] std::size_t operator()(IncrementalTopologyEdgeId id) const noexcept;
};

struct IncrementalTopologyTileIndex3DHash {
  [[nodiscard]] std::size_t
  operator()(IncrementalTopologyTileIndex3D index) const noexcept;
};

struct IncrementalTopologyNodeTraits3D {
  bool junction{false};
  bool turn{false};
  bool vertical_connector{false};
  bool frontier{false};
  bool terminal{false};
};

struct IncrementalTopologyNode3D {
  IncrementalTopologyNodeId id{};
  IncrementalTopologyTileIndex3D tile{};
  Point3 representative{};
  std::size_t support_cell_count{0U};
  std::size_t degree{0U};
  std::uint64_t geometry_revision{0U};
  std::uint64_t classification_revision{0U};
  IncrementalTopologyNodeTraits3D traits{};
  std::optional<ObservationFrontier> observation_frontier;
};

struct IncrementalTopologyEdge3D {
  IncrementalTopologyEdgeId id{};
  IncrementalTopologyNodeId first{};
  IncrementalTopologyNodeId second{};
  Point3 first_contact{};
  Point3 second_contact{};
  double length_m{0.0};
  std::uint64_t supporting_revision{0U};
};

struct IncrementalTopologyGraph3DConfig {
  int tile_size_cells{8};
  int coarse_sample_stride_cells{2};
  int refined_sample_stride_cells{1};
  std::size_t maximum_observed_tiles_per_update{64U};
  std::size_t maximum_frontier_evaluations_per_component{128U};
  SweptFootprintConfig footprint{};
  SensorObservabilityConfig observability{};
};

struct IncrementalTopologyGraph3DUpdate {
  std::uint64_t revision{0U};
  std::size_t requested_dirty_chunks{0U};
  std::size_t discovered_dirty_tiles{0U};
  std::size_t rebuilt_tiles{0U};
  std::size_t pending_tiles{0U};
  std::size_t adaptively_refined_tiles{0U};
  std::size_t sampled_navigable_cells{0U};
  std::size_t retained_node_ids{0U};
  std::size_t created_nodes{0U};
  std::size_t retired_nodes{0U};
  std::size_t node_count{0U};
  std::size_t edge_count{0U};
  bool full_reset{false};
};

class IncrementalTopologyGraph3DSnapshot {
public:
  [[nodiscard]] std::uint64_t revision() const noexcept;
  [[nodiscard]] const GridBounds3D& bounds() const noexcept;
  [[nodiscard]] std::span<const IncrementalTopologyNode3D> nodes() const noexcept;
  [[nodiscard]] std::span<const IncrementalTopologyEdge3D> edges() const noexcept;
  [[nodiscard]] const IncrementalTopologyNode3D*
  findNode(IncrementalTopologyNodeId id) const noexcept;
  [[nodiscard]] std::optional<IncrementalTopologyNodeId>
  nodeForSampleCell(GridIndex3D cell) const noexcept;
  [[nodiscard]] std::optional<IncrementalTopologyNodeId>
  nearestNode(const Point3& position, double maximum_distance_m) const noexcept;

private:
  friend class IncrementalTopologyGraph3D;

  std::uint64_t revision_{0U};
  GridBounds3D bounds_{};
  std::vector<IncrementalTopologyNode3D> nodes_;
  std::vector<IncrementalTopologyEdge3D> edges_;
  std::unordered_map<IncrementalTopologyNodeId, std::size_t,
                     IncrementalTopologyNodeIdHash>
      node_indices_;
  std::unordered_map<std::uint64_t, IncrementalTopologyNodeId> sample_cell_nodes_;
};

class IncrementalTopologyGraph3D {
public:
  explicit IncrementalTopologyGraph3D(IncrementalTopologyGraph3DConfig config = {});
  ~IncrementalTopologyGraph3D();

  IncrementalTopologyGraph3D(IncrementalTopologyGraph3D&&) noexcept;
  IncrementalTopologyGraph3D& operator=(IncrementalTopologyGraph3D&&) noexcept;
  IncrementalTopologyGraph3D(const IncrementalTopologyGraph3D&) = delete;
  IncrementalTopologyGraph3D& operator=(const IncrementalTopologyGraph3D&) = delete;

  [[nodiscard]] IncrementalTopologyGraph3DUpdate
  update(const ObservedOccupancyGrid3D& occupancy, std::uint64_t revision,
         std::span<const OccupancyChunkIndex3D> dirty_chunks, bool full_reset);

  [[nodiscard]] IncrementalTopologyGraph3DUpdate reset(const OccupancyGrid3D& occupancy,
                                                       std::uint64_t revision);

  [[nodiscard]] IncrementalTopologyGraph3DSnapshot snapshot() const;
  [[nodiscard]] const IncrementalTopologyGraph3DConfig& config() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] bool incrementalTopologyGraph3DConfigIsValid(
    const IncrementalTopologyGraph3DConfig& config) noexcept;

} // namespace drone_city_nav
