#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <chrono>
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

struct IncrementalTopologyBlockIndex3D {
  int x{0};
  int y{0};
  int z{0};

  [[nodiscard]] auto
  operator<=>(const IncrementalTopologyBlockIndex3D&) const noexcept = default;
};

struct IncrementalTopologyNodeIdHash {
  [[nodiscard]] std::size_t operator()(IncrementalTopologyNodeId id) const noexcept;
};

struct IncrementalTopologyEdgeIdHash {
  [[nodiscard]] std::size_t operator()(IncrementalTopologyEdgeId id) const noexcept;
};

struct IncrementalTopologyBlockIndex3DHash {
  [[nodiscard]] std::size_t
  operator()(IncrementalTopologyBlockIndex3D index) const noexcept;
};

enum class IncrementalTopologyLineageEvent3D : std::uint8_t {
  kCreated,
  kRetained,
  kSplit,
  kMerge,
};

enum class IncrementalTopologyTransitionKind3D : std::uint8_t {
  kObservedFree,
  kOptimisticUnknown,
};

struct IncrementalTopologyTransitionEvidence3D {
  IncrementalTopologyTransitionKind3D kind{
      IncrementalTopologyTransitionKind3D::kObservedFree};
  std::size_t support_segment_count{0U};
  std::uint64_t validated_through_revision{0U};
  std::uint64_t complete_through_revision{0U};
  std::uint64_t lineage_id{0U};
  bool unknown_exposure{false};
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
  IncrementalTopologyBlockIndex3D block{};
  Point3 representative{};
  std::size_t support_cell_count{0U};
  std::size_t degree{0U};
  std::uint64_t created_on_revision{0U};
  std::uint64_t validated_through_revision{0U};
  std::uint64_t complete_through_revision{0U};
  std::uint64_t generation{0U};
  std::uint64_t classification_revision{0U};
  IncrementalTopologyLineageEvent3D lineage_event{
      IncrementalTopologyLineageEvent3D::kCreated};
  std::vector<IncrementalTopologyNodeId> predecessors;
  bool unknown_boundary_exposure{false};
  IncrementalTopologyNodeTraits3D traits{};
};

struct IncrementalTopologyEdge3D {
  IncrementalTopologyEdgeId id{};
  IncrementalTopologyNodeId first{};
  IncrementalTopologyNodeId second{};
  Point3 first_contact{};
  Point3 second_contact{};
  std::vector<Point3> polyline;
  double length_m{0.0};
  std::uint64_t created_on_revision{0U};
  IncrementalTopologyTransitionEvidence3D evidence{};
};

struct IncrementalTopologyBlockCoverage3D {
  IncrementalTopologyBlockIndex3D block{};
  std::uint64_t validated_through_revision{0U};
  std::uint64_t complete_through_revision{0U};
  bool pending_rebuild{false};
};

struct IncrementalTopologyGraph3DConfig {
  int block_size_cells{16};
  int coarse_sample_stride_cells{2};
  int refined_sample_stride_cells{1};
  std::size_t maximum_observed_blocks_per_update{16U};
  std::size_t maximum_backlog_blocks_per_update{64U};
  std::size_t backlog_boost_threshold_blocks{256U};
  std::size_t minimum_oldest_blocks_per_update{4U};
  double local_priority_radius_m{12.0};
  double forward_corridor_radius_m{8.0};
  double forward_corridor_lookahead_m{60.0};
  SweptFootprintConfig footprint{};
  bool require_known_free_space{false};
};

struct IncrementalTopologyGraph3DUpdate {
  std::uint64_t revision{0U};
  std::uint64_t source_seen_revision{0U};
  std::uint64_t materialized_revision{0U};
  std::uint64_t coverage_complete_through_revision{0U};
  std::size_t requested_dirty_chunks{0U};
  std::size_t discovered_dirty_blocks{0U};
  std::size_t rebuilt_blocks{0U};
  std::size_t refreshed_observation_blocks{0U};
  std::size_t pending_blocks{0U};
  std::size_t scheduled_block_budget{0U};
  std::size_t local_priority_blocks{0U};
  std::size_t forward_corridor_blocks{0U};
  std::size_t oldest_preserved_blocks{0U};
  std::size_t adaptively_refined_blocks{0U};
  std::size_t sampled_navigable_cells{0U};
  std::size_t retained_node_ids{0U};
  std::size_t created_nodes{0U};
  std::size_t retired_nodes{0U};
  std::size_t node_count{0U};
  std::size_t edge_count{0U};
  double dirty_block_discovery_ms{0.0};
  double block_build_ms{0.0};
  double block_replace_ms{0.0};
  double block_connect_ms{0.0};
  double node_classification_ms{0.0};
  double graph_rebuild_ms{0.0};
  bool full_reset{false};
  bool backlog_boosted{false};
  bool deadline_exhausted{false};
  bool minimum_progress_guaranteed{false};
};

struct IncrementalTopologyBuildPriority3D {
  Point3 position{};
  Point3 target{};
  double local_radius_m{12.0};
  double forward_corridor_radius_m{8.0};
  double forward_corridor_lookahead_m{60.0};
};

struct IncrementalTopologyConnector3D {
  IncrementalTopologyNodeId node{};
  std::vector<Point3> polyline;
  double length_m{0.0};
  IncrementalTopologyTransitionEvidence3D evidence{};
};

struct IncrementalTopologySampleRecord3D {
  GridIndex3D cell{};
  GridIndex3D parent_cell{};
  IncrementalTopologyNodeId node{};
};

struct IncrementalTopologySampleBlock3D {
  IncrementalTopologyBlockIndex3D block{};
  std::vector<IncrementalTopologySampleRecord3D> records;
};

class IncrementalTopologyGraph3DSnapshot {
public:
  [[nodiscard]] std::uint64_t revision() const noexcept;
  [[nodiscard]] std::uint64_t sourceSeenRevision() const noexcept;
  [[nodiscard]] std::uint64_t materializedRevision() const noexcept;
  [[nodiscard]] std::uint64_t coverageCompleteThroughRevision() const noexcept;
  [[nodiscard]] const GridBounds3D& bounds() const noexcept;
  [[nodiscard]] std::span<const IncrementalTopologyNode3D> nodes() const noexcept;
  [[nodiscard]] std::span<const IncrementalTopologyEdge3D> edges() const noexcept;
  [[nodiscard]] std::span<const IncrementalTopologyBlockCoverage3D>
  blockCoverage() const noexcept;
  [[nodiscard]] std::size_t pendingBlockCount() const noexcept;
  [[nodiscard]] std::size_t sampleBlockCount() const noexcept;
  [[nodiscard]] std::size_t sampleCount() const noexcept;
  [[nodiscard]] const IncrementalTopologyNode3D*
  findNode(IncrementalTopologyNodeId id) const noexcept;
  [[nodiscard]] std::optional<IncrementalTopologyNodeId>
  nodeForSampleCell(GridIndex3D cell) const noexcept;
  [[nodiscard]] std::optional<IncrementalTopologyNodeId>
  nearestNode(const Point3& position, double maximum_distance_m) const noexcept;
  [[nodiscard]] std::optional<IncrementalTopologyConnector3D>
  connectObserved(const ObservedOccupancyGrid3D& occupancy, const Point3& position,
                  double maximum_distance_m, const SweptFootprintConfig& footprint,
                  ObservedSpaceValidationPolicy validation_policy) const;
  [[nodiscard]] std::optional<IncrementalTopologyConnector3D>
  connectObservedSample(const ObservedOccupancyGrid3D& occupancy, GridIndex3D cell,
                        const SweptFootprintConfig& footprint,
                        ObservedSpaceValidationPolicy validation_policy) const;

private:
  friend class IncrementalTopologyGraph3D;

  std::uint64_t revision_{0U};
  std::uint64_t source_seen_revision_{0U};
  std::uint64_t materialized_revision_{0U};
  std::uint64_t coverage_complete_through_revision_{0U};
  GridBounds3D bounds_{};
  std::vector<IncrementalTopologyNode3D> nodes_;
  std::vector<IncrementalTopologyEdge3D> edges_;
  std::vector<IncrementalTopologyBlockCoverage3D> block_coverage_;
  std::size_t pending_block_count_{0U};
  std::size_t sample_count_{0U};
  std::unordered_map<IncrementalTopologyNodeId, std::size_t,
                     IncrementalTopologyNodeIdHash>
      node_indices_;
  int sample_block_size_cells_{1};
  std::vector<std::shared_ptr<const IncrementalTopologySampleBlock3D>> sample_blocks_;
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
         std::span<const OccupancyChunkIndex3D> dirty_chunks, bool full_reset,
         std::optional<IncrementalTopologyBuildPriority3D> priority = std::nullopt,
         std::optional<std::chrono::steady_clock::time_point> deadline = std::nullopt);

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

[[nodiscard]] const char* incrementalTopologyTransitionKind3DName(
    IncrementalTopologyTransitionKind3D kind) noexcept;

} // namespace drone_city_nav
