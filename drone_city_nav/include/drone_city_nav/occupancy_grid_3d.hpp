#pragma once

#include "drone_city_nav/types.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace drone_city_nav {

struct OccupancyChunkIndex3D {
  int x{0};
  int y{0};
  int z{0};

  [[nodiscard]] bool operator==(const OccupancyChunkIndex3D&) const noexcept = default;
};

struct OccupancyChunkIndex3DHash {
  [[nodiscard]] std::size_t
  operator()(const OccupancyChunkIndex3D& index) const noexcept;
};

class OccupancyGrid3D {
public:
  static constexpr int kChunkSize{16};
  static constexpr std::size_t kVoxelsPerChunk{
      static_cast<std::size_t>(kChunkSize * kChunkSize * kChunkSize)};
  static constexpr std::size_t kWordsPerChunk{(kVoxelsPerChunk + 63U) / 64U};
  using Chunk = std::array<std::uint64_t, kWordsPerChunk>;

  explicit OccupancyGrid3D(const GridBounds3D& bounds, std::uint64_t fingerprint = 0U);

  [[nodiscard]] static OccupancyGrid3D load(const std::filesystem::path& path);
  void write(const std::filesystem::path& path) const;

  [[nodiscard]] const GridBounds3D& bounds() const noexcept;
  [[nodiscard]] std::uint64_t fingerprint() const noexcept;
  [[nodiscard]] std::size_t occupiedChunkCount() const noexcept;
  [[nodiscard]] std::size_t occupiedVoxelCount() const noexcept;
  [[nodiscard]] bool contains(GridIndex3D index) const noexcept;
  [[nodiscard]] std::optional<GridIndex3D>
  worldToCell(const Point3& point) const noexcept;
  [[nodiscard]] Point3 cellCenter(GridIndex3D index) const noexcept;
  [[nodiscard]] bool isOccupied(GridIndex3D index) const noexcept;

  void setOccupied(GridIndex3D index);

private:
  [[nodiscard]] static OccupancyChunkIndex3D chunkIndex(GridIndex3D index) noexcept;
  [[nodiscard]] static std::size_t localBitIndex(GridIndex3D index) noexcept;

  GridBounds3D bounds_{};
  std::uint64_t fingerprint_{0U};
  std::size_t occupied_voxels_{0U};
  std::unordered_map<OccupancyChunkIndex3D, Chunk, OccupancyChunkIndex3DHash> chunks_;
};

} // namespace drone_city_nav
