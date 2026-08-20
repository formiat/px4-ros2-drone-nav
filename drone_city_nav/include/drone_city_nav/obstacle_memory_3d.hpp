#pragma once

#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/tracked_agent_lidar_filter.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {

struct LidarBeam3D {
  Vec3 direction_map{};
  double range_m{0.0};
  bool hit{false};
  bool valid{false};
};

struct LidarScan3DView {
  Point3 origin_map{};
  std::span<const LidarBeam3D> beams{};
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
};

struct ObstacleMemory3DStats {
  std::size_t source_beams{0U};
  std::size_t processed_beams{0U};
  std::size_t hit_beams{0U};
  std::size_t miss_beams{0U};
  std::size_t invalid_beams{0U};
  std::size_t free_voxel_updates{0U};
  std::size_t occupied_voxel_updates{0U};
  std::size_t state_transitions{0U};
  std::size_t outside_endpoints{0U};
};

struct ObstacleMemory3DChanges {
  std::uint64_t revision{0U};
  std::vector<OccupancyChunkIndex3D> dirty_chunks;
  bool full_reset{false};
};

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

private:
  struct EvidenceChunk {
    std::array<std::int16_t, OccupancyGrid3D::kVoxelsPerChunk> scores{};
  };

  [[nodiscard]] bool applyEvidence(GridIndex3D index, int delta,
                                   ObstacleMemory3DStats& stats);
  void integrateRay(const Point3& origin, const LidarBeam3D& beam,
                    ObstacleMemory3DStats& stats);

  ObstacleMemory3DConfig config_{};
  ObservedOccupancyGrid3D grid_;
  std::unordered_map<OccupancyChunkIndex3D, EvidenceChunk, OccupancyChunkIndex3DHash>
      evidence_;
  std::unordered_map<OccupancyChunkIndex3D, bool, OccupancyChunkIndex3DHash>
      dirty_chunks_;
  std::uint64_t revision_{0U};
  bool full_reset_pending_{true};
};

} // namespace drone_city_nav
