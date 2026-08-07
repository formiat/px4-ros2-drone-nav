#pragma once

#include "drone_city_nav/distance_field_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>

namespace drone_city_nav {

struct StaticEsdfCacheExtractionStats {
  std::size_t requested_chunks{0U};
  std::size_t decoded_chunks{0U};
  std::size_t decoded_chunk_cache_hits{0U};
  std::size_t resident_decoded_chunks{0U};
  std::size_t finite_voxels{0U};
  double decode_ms{0.0};
  double copy_ms{0.0};
  double duration_ms{0.0};
};

struct StaticEsdfCacheExtraction {
  DistanceField3D field;
  StaticEsdfCacheExtractionStats stats;
};

class StaticEsdfCache {
public:
  static constexpr int kChunkSize{16};

  [[nodiscard]] static StaticEsdfCache load(const std::filesystem::path& path);
  static void write(const std::filesystem::path& path, const OccupancyGrid3D& occupancy,
                    const DistanceField3D& global_field, int compression_level = 9);
  [[nodiscard]] static GridBounds3D
  alignRegionToChunks(const GridBounds3D& world_bounds,
                      const GridBounds3D& requested_bounds);

  [[nodiscard]] bool compatibleWith(const OccupancyGrid3D& occupancy,
                                    double requested_maximum_distance_m) const noexcept;
  [[nodiscard]] StaticEsdfCacheExtraction extract(const GridBounds3D& local_bounds,
                                                  double maximum_distance_m) const;

  [[nodiscard]] const GridBounds3D& bounds() const noexcept;
  [[nodiscard]] std::uint64_t occupancyFingerprint() const noexcept;
  [[nodiscard]] double maximumDistanceM() const noexcept;
  [[nodiscard]] std::size_t storedChunkCount() const noexcept;
  [[nodiscard]] std::size_t compressedBytes() const noexcept;
  [[nodiscard]] bool sharedResourceReused() const noexcept;

private:
  struct ChunkRecord {
    std::size_t payload_offset{0U};
    std::size_t payload_size{0U};
    std::uint64_t checksum{0U};
  };

  struct SharedStorage;

  GridBounds3D bounds_{};
  std::uint64_t occupancy_fingerprint_{0U};
  double maximum_distance_m_{0.0};
  std::shared_ptr<SharedStorage> shared_storage_;
  bool shared_resource_reused_{false};
};

} // namespace drone_city_nav
