#include "drone_city_nav/static_esdf_cache.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>
#include <zstd.h>

namespace drone_city_nav {
namespace {

constexpr std::array<char, 8U> kMagic{'D', 'C', 'N', 'E', 'S', 'F', '3', 'D'};
constexpr std::uint32_t kVersion{1U};
constexpr std::uint16_t kInfinity = std::numeric_limits<std::uint16_t>::max();
constexpr std::size_t kValuesPerChunk{
    static_cast<std::size_t>(StaticEsdfCache::kChunkSize) *
    static_cast<std::size_t>(StaticEsdfCache::kChunkSize) *
    static_cast<std::size_t>(StaticEsdfCache::kChunkSize)};
constexpr std::size_t kDecodedChunkBytes{kValuesPerChunk * sizeof(std::uint16_t)};
constexpr std::uint64_t kFnvOffset{14695981039346656037ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};

template<typename Value> void writeValue(std::ostream& stream, const Value& value) {
  static_assert(std::is_trivially_copyable_v<Value>);
  // Binary world artifacts intentionally expose trivially-copyable storage.
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream.write(reinterpret_cast<const char*>(&value),
               static_cast<std::streamsize>(sizeof(Value)));
  if (!stream) {
    throw std::runtime_error{"failed to write static ESDF cache"};
  }
}

class ByteReader {
public:
  explicit ByteReader(const std::span<const std::byte> bytes)
      : bytes_{bytes} {
  }

  template<typename Value> [[nodiscard]] Value read(const char* description) {
    static_assert(std::is_trivially_copyable_v<Value>);
    if (offset_ > bytes_.size() || sizeof(Value) > bytes_.size() - offset_) {
      throw std::runtime_error{
          std::string{"truncated static ESDF cache while reading "} + description};
    }
    Value value{};
    std::memcpy(&value, bytes_.data() + offset_, sizeof(Value));
    offset_ += sizeof(Value);
    return value;
  }

  [[nodiscard]] std::span<const std::byte> readBytes(const std::size_t size,
                                                     const char* description) {
    if (offset_ > bytes_.size() || size > bytes_.size() - offset_) {
      throw std::runtime_error{
          std::string{"truncated static ESDF cache while reading "} + description};
    }
    const std::span result = bytes_.subspan(offset_, size);
    offset_ += size;
    return result;
  }

  [[nodiscard]] std::size_t offset() const noexcept {
    return offset_;
  }

  [[nodiscard]] bool atEnd() const noexcept {
    return offset_ == bytes_.size();
  }

private:
  std::span<const std::byte> bytes_;
  std::size_t offset_{0U};
};

[[nodiscard]] std::uint64_t checksum(const std::span<const std::byte> bytes) noexcept {
  std::uint64_t result{kFnvOffset};
  for (const std::byte value : bytes) {
    result ^= static_cast<std::uint64_t>(std::to_integer<unsigned char>(value));
    result *= kFnvPrime;
  }
  return result;
}

[[nodiscard]] bool sameBounds(const GridBounds3D& lhs,
                              const GridBounds3D& rhs) noexcept {
  constexpr double tolerance = 1.0e-6;
  return std::abs(lhs.origin_x - rhs.origin_x) <= tolerance &&
         std::abs(lhs.origin_y - rhs.origin_y) <= tolerance &&
         std::abs(lhs.origin_z - rhs.origin_z) <= tolerance &&
         std::abs(lhs.resolution_m - rhs.resolution_m) <= tolerance &&
         lhs.width_cells == rhs.width_cells && lhs.height_cells == rhs.height_cells &&
         lhs.depth_cells == rhs.depth_cells;
}

[[nodiscard]] std::size_t checkedVoxelCount(const GridBounds3D& bounds) {
  const std::size_t width = static_cast<std::size_t>(bounds.width_cells);
  const std::size_t height = static_cast<std::size_t>(bounds.height_cells);
  const std::size_t depth = static_cast<std::size_t>(bounds.depth_cells);
  if (height != 0U && width > std::numeric_limits<std::size_t>::max() / height) {
    throw std::overflow_error{"static ESDF cache dimensions overflow"};
  }
  const std::size_t plane = width * height;
  if (depth != 0U && plane > std::numeric_limits<std::size_t>::max() / depth) {
    throw std::overflow_error{"static ESDF cache dimensions overflow"};
  }
  return plane * depth;
}

[[nodiscard]] int alignedOffset(const double local_origin, const double world_origin,
                                const double resolution) {
  const double cells = (local_origin - world_origin) / resolution;
  const auto rounded = static_cast<int>(std::llround(cells));
  if (std::abs(cells - static_cast<double>(rounded)) > 1.0e-6) {
    throw std::invalid_argument{"static ESDF cache ROI is not grid aligned"};
  }
  return rounded;
}

[[nodiscard]] std::size_t localIndex(const GridBounds3D& bounds, const int x,
                                     const int y, const int z) noexcept {
  return (static_cast<std::size_t>(z) * static_cast<std::size_t>(bounds.height_cells) +
          static_cast<std::size_t>(y)) *
             static_cast<std::size_t>(bounds.width_cells) +
         static_cast<std::size_t>(x);
}

[[nodiscard]] std::size_t chunkIndex(const int x, const int y, const int z) noexcept {
  return (static_cast<std::size_t>(z) *
              static_cast<std::size_t>(StaticEsdfCache::kChunkSize) +
          static_cast<std::size_t>(y)) *
             static_cast<std::size_t>(StaticEsdfCache::kChunkSize) +
         static_cast<std::size_t>(x);
}

[[nodiscard]] std::vector<std::byte> readFile(const std::filesystem::path& path) {
  std::ifstream stream{path, std::ios::binary | std::ios::ate};
  if (!stream) {
    throw std::runtime_error{"failed to open static ESDF cache: " + path.string()};
  }
  const std::streamsize size = stream.tellg();
  if (size <= 0) {
    throw std::runtime_error{"empty static ESDF cache: " + path.string()};
  }
  stream.seekg(0);
  std::vector<std::byte> bytes(static_cast<std::size_t>(size));
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream.read(reinterpret_cast<char*>(bytes.data()), size);
  if (!stream) {
    throw std::runtime_error{"failed to read static ESDF cache: " + path.string()};
  }
  return bytes;
}

} // namespace

struct StaticEsdfCache::SharedStorage {
  GridBounds3D bounds{};
  std::uint64_t occupancy_fingerprint{0U};
  double maximum_distance_m{0.0};
  std::vector<std::byte> compressed_storage;
  std::unordered_map<OccupancyChunkIndex3D, ChunkRecord, OccupancyChunkIndex3DHash>
      chunks;
  mutable std::shared_mutex decoded_mutex;
  mutable std::unordered_map<OccupancyChunkIndex3D,
                             std::shared_ptr<const std::vector<float>>,
                             OccupancyChunkIndex3DHash>
      decoded_chunks;
};

StaticEsdfCache StaticEsdfCache::load(const std::filesystem::path& path) {
  const std::filesystem::path canonical_path = std::filesystem::weakly_canonical(path);
  const std::string registry_key =
      canonical_path.string() + ':' + std::to_string(std::filesystem::file_size(path)) +
      ':' +
      std::to_string(std::filesystem::last_write_time(path).time_since_epoch().count());
  static std::mutex registry_mutex;
  static std::unordered_map<std::string, std::weak_ptr<SharedStorage>> registry;
  const std::scoped_lock registry_lock{registry_mutex};

  StaticEsdfCache cache;
  if (const auto found = registry.find(registry_key); found != registry.end()) {
    cache.shared_storage_ = found->second.lock();
  }
  if (cache.shared_storage_) {
    cache.bounds_ = cache.shared_storage_->bounds;
    cache.occupancy_fingerprint_ = cache.shared_storage_->occupancy_fingerprint;
    cache.maximum_distance_m_ = cache.shared_storage_->maximum_distance_m;
    cache.shared_resource_reused_ = true;
    return cache;
  }

  auto storage = std::make_shared<SharedStorage>();
  storage->compressed_storage = readFile(path);
  ByteReader reader{storage->compressed_storage};
  std::array<char, 8U> magic{};
  const auto magic_bytes = reader.readBytes(magic.size(), "magic");
  std::memcpy(magic.data(), magic_bytes.data(), magic.size());
  const std::uint32_t version = reader.read<std::uint32_t>("version");
  const std::uint32_t chunk_size = reader.read<std::uint32_t>("chunk size");
  if (magic != kMagic || version != kVersion ||
      chunk_size != static_cast<std::uint32_t>(kChunkSize)) {
    throw std::runtime_error{"unsupported static ESDF cache format"};
  }
  storage->bounds.resolution_m = reader.read<float>("resolution");
  storage->bounds.origin_x = reader.read<float>("origin x");
  storage->bounds.origin_y = reader.read<float>("origin y");
  storage->bounds.origin_z = reader.read<float>("origin z");
  storage->bounds.width_cells = static_cast<int>(reader.read<std::uint32_t>("width"));
  storage->bounds.height_cells = static_cast<int>(reader.read<std::uint32_t>("height"));
  storage->bounds.depth_cells = static_cast<int>(reader.read<std::uint32_t>("depth"));
  storage->occupancy_fingerprint = reader.read<std::uint64_t>("occupancy fingerprint");
  storage->maximum_distance_m = reader.read<float>("maximum distance");
  const std::uint32_t chunk_count = reader.read<std::uint32_t>("chunk count");
  if (!(storage->bounds.resolution_m > 0.0) || storage->bounds.width_cells <= 0 ||
      storage->bounds.height_cells <= 0 || storage->bounds.depth_cells <= 0 ||
      !(storage->maximum_distance_m > 0.0)) {
    throw std::runtime_error{"invalid static ESDF cache metadata"};
  }
  storage->chunks.reserve(chunk_count);
  for (std::uint32_t number = 0U; number < chunk_count; ++number) {
    const OccupancyChunkIndex3D index{reader.read<std::int32_t>("chunk x"),
                                      reader.read<std::int32_t>("chunk y"),
                                      reader.read<std::int32_t>("chunk z")};
    const std::uint32_t payload_size = reader.read<std::uint32_t>("payload size");
    const std::uint64_t payload_checksum = reader.read<std::uint64_t>("checksum");
    if (payload_size == 0U) {
      throw std::runtime_error{"empty static ESDF cache chunk"};
    }
    const std::size_t payload_offset = reader.offset();
    static_cast<void>(reader.readBytes(payload_size, "chunk payload"));
    const auto [unused, inserted] = storage->chunks.emplace(
        index, ChunkRecord{payload_offset, payload_size, payload_checksum});
    static_cast<void>(unused);
    if (!inserted) {
      throw std::runtime_error{"duplicate static ESDF cache chunk"};
    }
  }
  if (!reader.atEnd()) {
    throw std::runtime_error{"trailing bytes in static ESDF cache"};
  }
  cache.bounds_ = storage->bounds;
  cache.occupancy_fingerprint_ = storage->occupancy_fingerprint;
  cache.maximum_distance_m_ = storage->maximum_distance_m;
  cache.shared_storage_ = storage;
  registry[registry_key] = storage;
  return cache;
}

void StaticEsdfCache::write(const std::filesystem::path& path,
                            const OccupancyGrid3D& occupancy,
                            const DistanceField3D& global_field,
                            const int compression_level) {
  if (!sameBounds(occupancy.bounds(), global_field.bounds()) ||
      !(global_field.maximumDistanceM() > 0.0)) {
    throw std::invalid_argument{"global field does not match static occupancy"};
  }
  const double maximum_cells =
      global_field.maximumDistanceM() / occupancy.bounds().resolution_m;
  if (maximum_cells * maximum_cells >= static_cast<double>(kInfinity)) {
    throw std::invalid_argument{"static ESDF cache maximum distance is too large"};
  }
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream{path, std::ios::binary | std::ios::trunc};
  if (!stream) {
    throw std::runtime_error{"failed to create static ESDF cache: " + path.string()};
  }
  stream.write(kMagic.data(), static_cast<std::streamsize>(kMagic.size()));
  writeValue(stream, kVersion);
  writeValue(stream, static_cast<std::uint32_t>(kChunkSize));
  writeValue(stream, static_cast<float>(occupancy.bounds().resolution_m));
  writeValue(stream, static_cast<float>(occupancy.bounds().origin_x));
  writeValue(stream, static_cast<float>(occupancy.bounds().origin_y));
  writeValue(stream, static_cast<float>(occupancy.bounds().origin_z));
  writeValue(stream, static_cast<std::uint32_t>(occupancy.bounds().width_cells));
  writeValue(stream, static_cast<std::uint32_t>(occupancy.bounds().height_cells));
  writeValue(stream, static_cast<std::uint32_t>(occupancy.bounds().depth_cells));
  writeValue(stream, occupancy.fingerprint());
  writeValue(stream, static_cast<float>(global_field.maximumDistanceM()));
  const std::streampos chunk_count_position = stream.tellp();
  writeValue(stream, std::uint32_t{0U});

  std::array<std::uint16_t, kValuesPerChunk> values{};
  std::vector<std::byte> compressed(ZSTD_compressBound(kDecodedChunkBytes));
  std::uint32_t stored_chunks{0U};
  const GridBounds3D& bounds = occupancy.bounds();
  const int chunks_x = (bounds.width_cells + kChunkSize - 1) / kChunkSize;
  const int chunks_y = (bounds.height_cells + kChunkSize - 1) / kChunkSize;
  const int chunks_z = (bounds.depth_cells + kChunkSize - 1) / kChunkSize;
  for (int chunk_z = 0; chunk_z < chunks_z; ++chunk_z) {
    for (int chunk_y = 0; chunk_y < chunks_y; ++chunk_y) {
      for (int chunk_x = 0; chunk_x < chunks_x; ++chunk_x) {
        values.fill(kInfinity);
        bool finite{false};
        for (int z = 0; z < kChunkSize; ++z) {
          const int global_z = chunk_z * kChunkSize + z;
          if (global_z >= bounds.depth_cells) {
            continue;
          }
          for (int y = 0; y < kChunkSize; ++y) {
            const int global_y = chunk_y * kChunkSize + y;
            if (global_y >= bounds.height_cells) {
              continue;
            }
            for (int x = 0; x < kChunkSize; ++x) {
              const int global_x = chunk_x * kChunkSize + x;
              if (global_x >= bounds.width_cells) {
                continue;
              }
              const float distance =
                  global_field.distanceAt(GridIndex3D{global_x, global_y, global_z});
              if (!std::isfinite(distance)) {
                continue;
              }
              const double cells = static_cast<double>(distance) / bounds.resolution_m;
              const auto squared =
                  static_cast<std::uint16_t>(std::llround(cells * cells));
              values.at(chunkIndex(x, y, z)) = squared;
              finite = true;
            }
          }
        }
        if (!finite) {
          continue;
        }
        const auto decoded = std::as_bytes(std::span{values});
        const std::size_t compressed_size =
            ZSTD_compress(compressed.data(), compressed.size(), decoded.data(),
                          decoded.size(), compression_level);
        if (ZSTD_isError(compressed_size) != 0U ||
            compressed_size > std::numeric_limits<std::uint32_t>::max()) {
          throw std::runtime_error{"failed to compress static ESDF cache chunk"};
        }
        writeValue(stream, static_cast<std::int32_t>(chunk_x));
        writeValue(stream, static_cast<std::int32_t>(chunk_y));
        writeValue(stream, static_cast<std::int32_t>(chunk_z));
        writeValue(stream, static_cast<std::uint32_t>(compressed_size));
        writeValue(stream, checksum(decoded));
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
        stream.write(reinterpret_cast<const char*>(compressed.data()),
                     static_cast<std::streamsize>(compressed_size));
        if (!stream) {
          throw std::runtime_error{"failed to write static ESDF cache payload"};
        }
        ++stored_chunks;
      }
    }
  }
  stream.seekp(chunk_count_position);
  writeValue(stream, stored_chunks);
}

GridBounds3D
StaticEsdfCache::alignRegionToChunks(const GridBounds3D& world_bounds,
                                     const GridBounds3D& requested_bounds) {
  if (!(world_bounds.resolution_m > 0.0) || world_bounds.width_cells <= 0 ||
      world_bounds.height_cells <= 0 || world_bounds.depth_cells <= 0 ||
      requested_bounds.width_cells <= 0 || requested_bounds.height_cells <= 0 ||
      requested_bounds.depth_cells <= 0 ||
      std::abs(requested_bounds.resolution_m - world_bounds.resolution_m) > 1.0e-6) {
    throw std::invalid_argument{"invalid static ESDF cache ROI bounds"};
  }
  const int offset_x = alignedOffset(requested_bounds.origin_x, world_bounds.origin_x,
                                     world_bounds.resolution_m);
  const int offset_y = alignedOffset(requested_bounds.origin_y, world_bounds.origin_y,
                                     world_bounds.resolution_m);
  const int offset_z = alignedOffset(requested_bounds.origin_z, world_bounds.origin_z,
                                     world_bounds.resolution_m);
  const auto align_axis = [](const int offset, const int extent,
                             const int world_extent) {
    if (offset < 0 || extent <= 0 || offset + extent > world_extent) {
      throw std::out_of_range{"static ESDF cache ROI is outside world bounds"};
    }
    const int first = (offset / kChunkSize) * kChunkSize;
    const int requested_end = offset + extent;
    const int aligned_end = std::min(
        world_extent, ((requested_end + kChunkSize - 1) / kChunkSize) * kChunkSize);
    return std::pair{first, aligned_end - first};
  };
  const auto [first_x, width] =
      align_axis(offset_x, requested_bounds.width_cells, world_bounds.width_cells);
  const auto [first_y, height] =
      align_axis(offset_y, requested_bounds.height_cells, world_bounds.height_cells);
  const auto [first_z, depth] =
      align_axis(offset_z, requested_bounds.depth_cells, world_bounds.depth_cells);
  return GridBounds3D{
      .origin_x = world_bounds.origin_x +
                  static_cast<double>(first_x) * world_bounds.resolution_m,
      .origin_y = world_bounds.origin_y +
                  static_cast<double>(first_y) * world_bounds.resolution_m,
      .origin_z = world_bounds.origin_z +
                  static_cast<double>(first_z) * world_bounds.resolution_m,
      .resolution_m = world_bounds.resolution_m,
      .width_cells = width,
      .height_cells = height,
      .depth_cells = depth,
  };
}

bool StaticEsdfCache::compatibleWith(
    const OccupancyGrid3D& occupancy,
    const double requested_maximum_distance_m) const noexcept {
  return sameBounds(bounds_, occupancy.bounds()) &&
         occupancy_fingerprint_ == occupancy.fingerprint() &&
         requested_maximum_distance_m > 0.0 &&
         requested_maximum_distance_m <= maximum_distance_m_ + 1.0e-6;
}

StaticEsdfCacheExtraction
StaticEsdfCache::extract(const GridBounds3D& local_bounds,
                         const double maximum_distance_m) const {
  const auto started = std::chrono::steady_clock::now();
  if (!shared_storage_ || !(maximum_distance_m > 0.0) ||
      maximum_distance_m > maximum_distance_m_ + 1.0e-6 ||
      std::abs(local_bounds.resolution_m - bounds_.resolution_m) > 1.0e-6) {
    throw std::invalid_argument{"static ESDF cache cannot satisfy requested field"};
  }
  const int offset_x =
      alignedOffset(local_bounds.origin_x, bounds_.origin_x, bounds_.resolution_m);
  const int offset_y =
      alignedOffset(local_bounds.origin_y, bounds_.origin_y, bounds_.resolution_m);
  const int offset_z =
      alignedOffset(local_bounds.origin_z, bounds_.origin_z, bounds_.resolution_m);
  if (offset_x < 0 || offset_y < 0 || offset_z < 0 ||
      offset_x + local_bounds.width_cells > bounds_.width_cells ||
      offset_y + local_bounds.height_cells > bounds_.height_cells ||
      offset_z + local_bounds.depth_cells > bounds_.depth_cells) {
    throw std::out_of_range{"static ESDF cache ROI is outside world bounds"};
  }

  StaticEsdfCacheExtraction result;
  result.field.bounds_ = local_bounds;
  result.field.maximum_distance_m_ = maximum_distance_m;
  result.field.distances_m_.assign(checkedVoxelCount(local_bounds),
                                   std::numeric_limits<float>::infinity());
  result.field.stats_.voxel_count = result.field.distances_m_.size();
  const int first_chunk_x = offset_x / kChunkSize;
  const int first_chunk_y = offset_y / kChunkSize;
  const int first_chunk_z = offset_z / kChunkSize;
  const int last_chunk_x = (offset_x + local_bounds.width_cells - 1) / kChunkSize;
  const int last_chunk_y = (offset_y + local_bounds.height_cells - 1) / kChunkSize;
  const int last_chunk_z = (offset_z + local_bounds.depth_cells - 1) / kChunkSize;
  for (int chunk_z = first_chunk_z; chunk_z <= last_chunk_z; ++chunk_z) {
    for (int chunk_y = first_chunk_y; chunk_y <= last_chunk_y; ++chunk_y) {
      for (int chunk_x = first_chunk_x; chunk_x <= last_chunk_x; ++chunk_x) {
        const OccupancyChunkIndex3D chunk{chunk_x, chunk_y, chunk_z};
        const auto record = shared_storage_->chunks.find(chunk);
        if (record == shared_storage_->chunks.end()) {
          continue;
        }
        ++result.stats.requested_chunks;
        std::shared_ptr<const std::vector<float>> values;
        {
          const std::shared_lock lock{shared_storage_->decoded_mutex};
          if (const auto found = shared_storage_->decoded_chunks.find(chunk);
              found != shared_storage_->decoded_chunks.end()) {
            values = found->second;
          }
        }
        if (values) {
          ++result.stats.decoded_chunk_cache_hits;
        } else {
          const std::unique_lock lock{shared_storage_->decoded_mutex};
          if (const auto found = shared_storage_->decoded_chunks.find(chunk);
              found != shared_storage_->decoded_chunks.end()) {
            values = found->second;
            ++result.stats.decoded_chunk_cache_hits;
          } else {
            const auto decode_started = std::chrono::steady_clock::now();
            std::array<std::uint16_t, kValuesPerChunk> squared_values{};
            const std::span payload =
                std::span<const std::byte>{shared_storage_->compressed_storage}.subspan(
                    record->second.payload_offset, record->second.payload_size);
            const std::size_t decoded_size =
                ZSTD_decompress(squared_values.data(), kDecodedChunkBytes,
                                payload.data(), payload.size());
            if (ZSTD_isError(decoded_size) != 0U ||
                decoded_size != kDecodedChunkBytes) {
              throw std::runtime_error{"failed to decompress static ESDF cache chunk"};
            }
            const auto decoded = std::as_bytes(std::span{squared_values});
            if (checksum(decoded) != record->second.checksum) {
              throw std::runtime_error{"static ESDF cache chunk checksum mismatch"};
            }
            auto decoded_values = std::make_shared<std::vector<float>>(
                kValuesPerChunk, std::numeric_limits<float>::infinity());
            for (std::size_t index = 0U; index < squared_values.size(); ++index) {
              if (squared_values.at(index) == kInfinity) {
                continue;
              }
              (*decoded_values)[index] = static_cast<float>(
                  std::sqrt(static_cast<double>(squared_values.at(index))) *
                  bounds_.resolution_m);
            }
            values = decoded_values;
            shared_storage_->decoded_chunks.emplace(chunk, std::move(decoded_values));
            ++result.stats.decoded_chunks;
            result.stats.decode_ms +=
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - decode_started)
                    .count();
          }
        }
        const auto copy_started = std::chrono::steady_clock::now();
        for (int z = 0; z < kChunkSize; ++z) {
          const int local_z = chunk_z * kChunkSize + z - offset_z;
          if (local_z < 0 || local_z >= local_bounds.depth_cells) {
            continue;
          }
          for (int y = 0; y < kChunkSize; ++y) {
            const int local_y = chunk_y * kChunkSize + y - offset_y;
            if (local_y < 0 || local_y >= local_bounds.height_cells) {
              continue;
            }
            for (int x = 0; x < kChunkSize; ++x) {
              const int local_x = chunk_x * kChunkSize + x - offset_x;
              if (local_x < 0 || local_x >= local_bounds.width_cells) {
                continue;
              }
              const float distance = values->at(chunkIndex(x, y, z));
              if (!std::isfinite(distance) ||
                  distance > static_cast<float>(maximum_distance_m)) {
                continue;
              }
              result.field
                  .distances_m_[localIndex(local_bounds, local_x, local_y, local_z)] =
                  distance;
              ++result.stats.finite_voxels;
            }
          }
        }
        result.stats.copy_ms += std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - copy_started)
                                    .count();
      }
    }
  }
  {
    const std::shared_lock lock{shared_storage_->decoded_mutex};
    result.stats.resident_decoded_chunks = shared_storage_->decoded_chunks.size();
  }
  result.stats.duration_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
  result.field.stats_.duration_ms = result.stats.duration_ms;
  result.field.stats_.finalize_ms = result.stats.copy_ms;
  return result;
}

const GridBounds3D& StaticEsdfCache::bounds() const noexcept {
  return bounds_;
}

std::uint64_t StaticEsdfCache::occupancyFingerprint() const noexcept {
  return occupancy_fingerprint_;
}

double StaticEsdfCache::maximumDistanceM() const noexcept {
  return maximum_distance_m_;
}

std::size_t StaticEsdfCache::storedChunkCount() const noexcept {
  return shared_storage_ ? shared_storage_->chunks.size() : 0U;
}

std::size_t StaticEsdfCache::compressedBytes() const noexcept {
  return shared_storage_ ? shared_storage_->compressed_storage.size() : 0U;
}

bool StaticEsdfCache::sharedResourceReused() const noexcept {
  return shared_resource_reused_;
}

} // namespace drone_city_nav
