#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {

struct LidarBeam3D {
  Vec3 direction_map{};
  double range_m{0.0};
  bool hit{false};
  bool valid{false};
  // A reconstructed surface sample: occupied evidence at the endpoint only.
  // No beam travelled to it, so it carries no free-space evidence.
  bool surface_only{false};
};

struct LidarScan3DView {
  Point3 origin_map{};
  std::span<const LidarBeam3D> beams{};
  // ROS acquisition time of this physical observation. A non-positive value
  // preserves the single-observation behaviour used by timestamp-less inputs.
  std::int64_t acquisition_stamp_ns{0};
};

struct ObstacleMemory3DConfig {
  double maximum_range_m{35.0};
  double minimum_range_m{0.2};
  int scan_stride{1};
  int hit_weight{4};
  int miss_weight{1};
  int minimum_score{-8};
  int maximum_score{12};
  int occupied_score{3};
  int free_score{-1};
  // Intervals are retained for diagnostics and stale-order rejection only.
  // Every integrated scan contributes exactly one physical observation.
  double nominal_evidence_interval_s{0.1};
  double maximum_evidence_interval_s{0.5};
  // Roadmap item 17 stage 8: an occupied voxel not confirmed for this long
  // per confirmation it has collected returns to unknown, which is free. A
  // wall seen a thousand times outlives any flight; a trail seen three times
  // is gone in seconds, and a surface the vehicle comes back to is seen and
  // confirmed again. Zero keeps every occupancy for the flight.
  double decay_seconds_per_confirmation{0.0};
};

struct ObstacleMemory3DStats {
  std::size_t source_beams{0U};
  std::size_t processed_beams{0U};
  std::size_t hit_beams{0U};
  std::size_t miss_beams{0U};
  std::size_t surface_beams{0U};
  std::size_t invalid_beams{0U};
  std::size_t free_voxel_updates{0U};
  std::size_t occupied_voxel_updates{0U};
  std::size_t state_transitions{0U};
  // Occupied voxels this scan returned to unknown for want of confirmation.
  std::size_t decayed_voxels{0U};
  std::size_t outside_endpoints{0U};
  double evidence_interval_s{0.0};
  bool stale_acquisition{false};
};

struct ObstacleMemory3DChanges {
  std::uint64_t revision{0U};
  std::vector<OccupancyChunkIndex3D> dirty_chunks;
  bool full_reset{false};
};

// The obstacle memory: a probabilistic 3D voxel occupancy grid (occupancy
// mapping in the manner of OctoMap), scored by hits and free-ray misses.
class ObstacleMemory3D {
public:
  ObstacleMemory3D(const GridBounds3D& bounds, ObstacleMemory3DConfig config = {});

  [[nodiscard]] ObstacleMemory3DStats integrateScan(const LidarScan3DView& scan);
  [[nodiscard]] std::size_t
  forgetDynamicVolumes(std::span<const DynamicAgentLidarVolume> volumes);
  void reset();

  [[nodiscard]] const ObservedOccupancyGrid3D& grid() const noexcept;
  [[nodiscard]] std::uint64_t revision() const noexcept;
  [[nodiscard]] ObstacleMemory3DChanges takeChanges();
  [[nodiscard]] std::size_t decayedVoxelTotal() const noexcept;

private:
  struct ScanEvidenceChunk {
    OccupancyGrid3D::Chunk observed{};
    OccupancyGrid3D::Chunk occupied{};
    // Hit by a return the sensor measured, not by a surface sample or the
    // dark it looked into (specification K14).
    OccupancyGrid3D::Chunk measured{};
  };

  using ScanEvidence = std::unordered_map<OccupancyChunkIndex3D, ScanEvidenceChunk,
                                          OccupancyChunkIndex3DHash>;

  // Consecutive cells of one ray share a chunk sixteen at a time; the cursor
  // keeps the chunk of the last recorded cell so the map is consulted only
  // when the ray crosses a chunk boundary.
  struct ScanEvidenceCursor {
    OccupancyChunkIndex3D chunk_index{};
    ScanEvidenceChunk* chunk{nullptr};
  };

  struct EvidenceChunk {
    std::array<double, OccupancyGrid3D::kVoxelsPerChunk> scores{};
    // How often each voxel was seen occupied, and when last, in milliseconds
    // since the memory's first stamped scan.
    std::array<std::uint16_t, OccupancyGrid3D::kVoxelsPerChunk> confirmations{};
    std::array<std::int32_t, OccupancyGrid3D::kVoxelsPerChunk> confirmed_at_ms{};
    // The earliest moment, on the same clock, an occupied voxel of the chunk
    // may decay.
    std::int64_t next_decay_ms{std::numeric_limits<std::int64_t>::max()};
  };

  // Applies one scan's evidence of a chunk to the scores and voxel states,
  // resolving the score chunk and the grid chunk once per chunk.
  void applyChunkEvidence(OccupancyChunkIndex3D chunk_index,
                          const ScanEvidenceChunk& chunk_evidence, std::int64_t now_ms,
                          ObstacleMemory3DStats& stats);
  // Returns the occupied voxels no longer confirmed to unknown.
  void decayUnconfirmed(std::int64_t now_ms, ObstacleMemory3DStats& stats);
  [[nodiscard]] double evidenceIntervalSeconds(const LidarScan3DView& scan,
                                               ObstacleMemory3DStats& stats);
  void recordScanEvidence(GridIndex3D index, bool occupied, bool measured,
                          ScanEvidence& scan_evidence,
                          ScanEvidenceCursor& cursor) const;
  void integrateRay(const Point3& origin, const LidarBeam3D& beam,
                    ScanEvidence& scan_evidence, ObstacleMemory3DStats& stats) const;
  [[nodiscard]] static GridIndex3D cellFromChunkBit(OccupancyChunkIndex3D chunk,
                                                    std::size_t bit_index) noexcept;

  ObstacleMemory3DConfig config_{};
  ObservedOccupancyGrid3D grid_;
  std::unordered_map<OccupancyChunkIndex3D, EvidenceChunk, OccupancyChunkIndex3DHash>
      evidence_;
  std::unordered_map<OccupancyChunkIndex3D, bool, OccupancyChunkIndex3DHash>
      dirty_chunks_;
  std::uint64_t revision_{0U};
  std::int64_t last_evidence_stamp_ns_{0};
  // The clock of the confirmations: the first stamped scan's time.
  std::int64_t decay_epoch_ns_{0};
  std::size_t decayed_voxel_total_{0U};
  bool full_reset_pending_{true};
};

} // namespace drone_city_nav
