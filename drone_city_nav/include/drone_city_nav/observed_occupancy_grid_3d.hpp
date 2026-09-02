#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>

namespace drone_city_nav {

enum class ObservedVoxelState : std::uint8_t {
  kUnknown = 0,
  kFree = 1,
  kOccupied = 2,
};

struct ObservedOccupancyChunk3D {
  OccupancyGrid3D::Chunk observed{};
  OccupancyGrid3D::Chunk occupied{};
};

class ObservedOccupancyChunkStorage3D {
public:
  ObservedOccupancyChunkStorage3D() = delete;

  [[nodiscard]] const ObservedOccupancyChunk3D& get() const noexcept;
  [[nodiscard]] const ObservedOccupancyChunk3D* operator->() const noexcept;

private:
  friend class ObservedOccupancyGrid3D;

  explicit ObservedOccupancyChunkStorage3D(ObservedOccupancyChunk3D chunk);
  [[nodiscard]] ObservedOccupancyChunk3D& mutableChunk();

  std::shared_ptr<ObservedOccupancyChunk3D> chunk_;
};

class ObservedOccupancyGrid3D {
public:
  static constexpr int kChunkSize{OccupancyGrid3D::kChunkSize};
  using Chunk = ObservedOccupancyChunk3D;
  using ChunkStorage = ObservedOccupancyChunkStorage3D;
  using ChunkMap = std::unordered_map<OccupancyChunkIndex3D, ChunkStorage,
                                      OccupancyChunkIndex3DHash>;

  explicit ObservedOccupancyGrid3D(const GridBounds3D& bounds);

  [[nodiscard]] const GridBounds3D& bounds() const noexcept;
  [[nodiscard]] bool contains(GridIndex3D index) const noexcept;
  [[nodiscard]] std::optional<GridIndex3D>
  worldToCell(const Point3& point) const noexcept;
  [[nodiscard]] Point3 cellCenter(GridIndex3D index) const noexcept;
  [[nodiscard]] ObservedVoxelState state(GridIndex3D index) const noexcept;
  [[nodiscard]] bool isKnown(GridIndex3D index) const noexcept;
  [[nodiscard]] bool isKnownFree(GridIndex3D index) const noexcept;
  [[nodiscard]] bool isOccupied(GridIndex3D index) const noexcept;
  [[nodiscard]] std::size_t knownVoxelCount() const noexcept;
  [[nodiscard]] std::size_t freeVoxelCount() const noexcept;
  [[nodiscard]] std::size_t occupiedVoxelCount() const noexcept;
  [[nodiscard]] const ChunkMap& chunks() const noexcept;
  [[nodiscard]] const Chunk* findChunk(OccupancyChunkIndex3D index) const noexcept;
  [[nodiscard]] static ObservedVoxelState chunkState(const Chunk& chunk,
                                                     std::size_t bit_index) noexcept;

  bool setState(GridIndex3D index, ObservedVoxelState state);
  bool replaceChunk(OccupancyChunkIndex3D index, const Chunk& chunk);
  void clear();

  [[nodiscard]] OccupancyGrid3D occupiedSnapshot() const;
  // Content fingerprint of the occupied evidence alone: identical to
  // occupiedSnapshot().contentFingerprint() without materializing the dense
  // snapshot. Cached until the next mutation.
  [[nodiscard]] std::uint64_t occupiedContentFingerprint() const;
  [[nodiscard]] ObservedOccupancyGrid3D crop(const GridBounds3D& bounds) const;

  [[nodiscard]] static OccupancyChunkIndex3D chunkIndex(GridIndex3D index) noexcept;
  [[nodiscard]] static std::size_t localBitIndex(GridIndex3D index) noexcept;

private:
  [[nodiscard]] static bool bit(const OccupancyGrid3D::Chunk& words,
                                std::size_t index) noexcept;
  static void setBit(OccupancyGrid3D::Chunk& words, std::size_t index,
                     bool value) noexcept;
  [[nodiscard]] Chunk& mutableChunk(OccupancyChunkIndex3D index);
  void updateCounts(ObservedVoxelState before, ObservedVoxelState after) noexcept;

  GridBounds3D bounds_{};
  std::size_t known_voxels_{0U};
  std::size_t free_voxels_{0U};
  std::size_t occupied_voxels_{0U};
  mutable std::optional<std::uint64_t> occupied_content_fingerprint_cache_;
  ChunkMap chunks_;
};

} // namespace drone_city_nav
