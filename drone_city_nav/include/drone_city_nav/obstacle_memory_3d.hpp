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
  // A return on a body the thermal core sights as warm (roadmap item 21,
  // specification K64): moving evidence.
  bool warm_body{false};
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
  // Roadmap item 21, specification K65: the moving layer. A measured hit the
  // thermal core marks a warm body's, or one in a voxel the memory holds
  // observed free with no ordinary occupancy beside it (an occupancy that
  // appeared where free space had been observed), is moving evidence: the
  // voxel is occupied at once, cleared by the first free ray through it,
  // gone unobserved after `moving_life_s`, and an ordinary occupancy once
  // it has been hit every scan for `moving_stand_s` (a body that stood).
  // Nought turns the layer off: every hit is an ordinary one.
  double moving_life_s{3.0};
  double moving_stand_s{5.0};
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
  // The moving layer (K65) this scan: hits held as moving, voxels cleared by
  // a free ray, gone unobserved, become ordinary; and the voxels it holds.
  std::size_t moving_hits{0U};
  std::size_t moving_cleared{0U};
  std::size_t moving_expired{0U};
  std::size_t moving_promoted{0U};
  // Ordinary occupancies handed to the layer by a neighbour that turned free
  // young (a body first seen where nothing had been observed).
  std::size_t moving_joined{0U};
  std::size_t moving_voxels{0U};
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
    // Hit as moving evidence (K65).
    OccupancyGrid3D::Chunk moving{};
  };

  // The moving layer's voxels of a chunk: when each was first held and last
  // hit, in milliseconds of the confirmations' clock.
  struct MovingChunk {
    OccupancyGrid3D::Chunk held{};
    std::array<std::int32_t, OccupancyGrid3D::kVoxelsPerChunk> since_ms{};
    std::array<std::int32_t, OccupancyGrid3D::kVoxelsPerChunk> last_ms{};
    std::size_t count{0U};
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
                          ScanEvidence& scan_evidence, ScanEvidenceCursor& cursor,
                          bool moving = false) const;
  // Whether a hit in this voxel is moving evidence by the voxel's state before
  // the scan (K65): the layer holds it, or it is observed free with no
  // ordinary occupancy among its 26 neighbours.
  [[nodiscard]] bool movingEvidenceAt(GridIndex3D cell) const noexcept;
  [[nodiscard]] bool heldMoving(GridIndex3D cell) const noexcept;
  [[nodiscard]] std::uint16_t confirmationsAt(GridIndex3D cell) const noexcept;
  // A young occupancy turned free: its young occupied neighbours are the
  // body that vacated it, held from now.
  void joinMoving(GridIndex3D cell, std::int64_t now_ms, ObstacleMemory3DStats& stats);
  // The layer's voxels after a scan: the expired to unknown, the stood to
  // ordinary occupancy.
  void settleMoving(std::int64_t now_ms, ObstacleMemory3DStats& stats);
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
  std::unordered_map<OccupancyChunkIndex3D, MovingChunk, OccupancyChunkIndex3DHash>
      moving_;
  std::uint64_t revision_{0U};
  std::int64_t last_evidence_stamp_ns_{0};
  // The clock of the confirmations: the first stamped scan's time.
  std::int64_t decay_epoch_ns_{0};
  std::size_t decayed_voxel_total_{0U};
  bool full_reset_pending_{true};
};

} // namespace drone_city_nav
