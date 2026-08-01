#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <bit>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace drone_city_nav {
namespace {

constexpr std::array<char, 8U> kMagic{'D', 'C', 'N', 'O', 'C', 'C', '3', 'D'};
constexpr std::uint32_t kVersion{1U};

template<typename Value>
[[nodiscard]] Value readValue(std::istream& stream, const char* description) {
  static_assert(std::is_trivially_copyable_v<Value>);
  Value value{};
  // Binary world artifacts intentionally expose trivially-copyable storage.
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream.read(reinterpret_cast<char*>(&value),
              static_cast<std::streamsize>(sizeof(Value)));
  if (!stream) {
    throw std::runtime_error{std::string{"truncated Occupancy3D while reading "} +
                             description};
  }
  return value;
}

[[nodiscard]] int floorDiv(const int value, const int divisor) noexcept {
  const int quotient = value / divisor;
  const int remainder = value % divisor;
  return remainder < 0 ? quotient - 1 : quotient;
}

[[nodiscard]] int positiveModulo(const int value, const int divisor) noexcept {
  const int result = value % divisor;
  return result < 0 ? result + divisor : result;
}

} // namespace

std::size_t OccupancyChunkIndex3DHash::operator()(
    const OccupancyChunkIndex3D& index) const noexcept {
  std::size_t seed = std::hash<int>{}(index.x);
  seed ^= std::hash<int>{}(index.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<int>{}(index.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  return seed;
}

OccupancyGrid3D::OccupancyGrid3D(const GridBounds3D& bounds,
                                 const std::uint64_t fingerprint)
    : bounds_{bounds},
      fingerprint_{fingerprint} {
  if (!(bounds_.resolution_m > 0.0) || bounds_.width_cells <= 0 ||
      bounds_.height_cells <= 0 || bounds_.depth_cells <= 0) {
    throw std::invalid_argument{"invalid Occupancy3D bounds"};
  }
}

OccupancyGrid3D OccupancyGrid3D::load(const std::filesystem::path& path) {
  std::ifstream stream{path, std::ios::binary};
  if (!stream) {
    throw std::runtime_error{"failed to open Occupancy3D: " + path.string()};
  }
  std::array<char, 8U> magic{};
  stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  if (!stream || magic != kMagic) {
    throw std::runtime_error{"invalid Occupancy3D magic: " + path.string()};
  }
  const std::uint32_t version = readValue<std::uint32_t>(stream, "version");
  const std::uint32_t chunk_size = readValue<std::uint32_t>(stream, "chunk size");
  if (version != kVersion || chunk_size != static_cast<std::uint32_t>(kChunkSize)) {
    throw std::runtime_error{"unsupported Occupancy3D version or chunk size"};
  }
  GridBounds3D bounds;
  bounds.resolution_m = static_cast<double>(readValue<float>(stream, "resolution"));
  bounds.origin_x = static_cast<double>(readValue<float>(stream, "origin x"));
  bounds.origin_y = static_cast<double>(readValue<float>(stream, "origin y"));
  bounds.origin_z = static_cast<double>(readValue<float>(stream, "origin z"));
  const std::uint32_t width = readValue<std::uint32_t>(stream, "width");
  const std::uint32_t height = readValue<std::uint32_t>(stream, "height");
  const std::uint32_t depth = readValue<std::uint32_t>(stream, "depth");
  if (width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      depth > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    throw std::runtime_error{"Occupancy3D dimensions exceed supported range"};
  }
  bounds.width_cells = static_cast<int>(width);
  bounds.height_cells = static_cast<int>(height);
  bounds.depth_cells = static_cast<int>(depth);
  const std::uint64_t fingerprint = readValue<std::uint64_t>(stream, "fingerprint");
  const std::uint32_t chunk_count = readValue<std::uint32_t>(stream, "chunk count");
  OccupancyGrid3D grid{bounds, fingerprint};
  grid.chunks_.reserve(chunk_count);
  for (std::uint32_t chunk_number = 0U; chunk_number < chunk_count; ++chunk_number) {
    const OccupancyChunkIndex3D index{
        readValue<std::int32_t>(stream, "chunk x"),
        readValue<std::int32_t>(stream, "chunk y"),
        readValue<std::int32_t>(stream, "chunk z"),
    };
    Chunk chunk{};
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    stream.read(reinterpret_cast<char*>(chunk.data()),
                static_cast<std::streamsize>(sizeof(chunk)));
    if (!stream) {
      throw std::runtime_error{"truncated Occupancy3D chunk payload"};
    }
    for (const std::uint64_t word : chunk) {
      grid.occupied_voxels_ += static_cast<std::size_t>(std::popcount(word));
    }
    const auto [unused, inserted] = grid.chunks_.emplace(index, chunk);
    static_cast<void>(unused);
    if (!inserted) {
      throw std::runtime_error{"duplicate Occupancy3D chunk"};
    }
  }
  if (stream.peek() != std::char_traits<char>::eof()) {
    throw std::runtime_error{"trailing data in Occupancy3D artifact"};
  }
  return grid;
}

const GridBounds3D& OccupancyGrid3D::bounds() const noexcept {
  return bounds_;
}

std::uint64_t OccupancyGrid3D::fingerprint() const noexcept {
  return fingerprint_;
}

std::size_t OccupancyGrid3D::occupiedChunkCount() const noexcept {
  return chunks_.size();
}

std::size_t OccupancyGrid3D::occupiedVoxelCount() const noexcept {
  return occupied_voxels_;
}

bool OccupancyGrid3D::contains(const GridIndex3D index) const noexcept {
  return index.x >= 0 && index.y >= 0 && index.z >= 0 &&
         index.x < bounds_.width_cells && index.y < bounds_.height_cells &&
         index.z < bounds_.depth_cells;
}

std::optional<GridIndex3D>
OccupancyGrid3D::worldToCell(const Point3& point) const noexcept {
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    return std::nullopt;
  }
  const GridIndex3D result{
      static_cast<int>(std::floor((point.x - bounds_.origin_x) / bounds_.resolution_m)),
      static_cast<int>(std::floor((point.y - bounds_.origin_y) / bounds_.resolution_m)),
      static_cast<int>(std::floor((point.z - bounds_.origin_z) / bounds_.resolution_m)),
  };
  return contains(result) ? std::optional<GridIndex3D>{result} : std::nullopt;
}

Point3 OccupancyGrid3D::cellCenter(const GridIndex3D index) const noexcept {
  return Point3{
      bounds_.origin_x + (static_cast<double>(index.x) + 0.5) * bounds_.resolution_m,
      bounds_.origin_y + (static_cast<double>(index.y) + 0.5) * bounds_.resolution_m,
      bounds_.origin_z + (static_cast<double>(index.z) + 0.5) * bounds_.resolution_m,
  };
}

OccupancyChunkIndex3D OccupancyGrid3D::chunkIndex(const GridIndex3D index) noexcept {
  return {floorDiv(index.x, kChunkSize), floorDiv(index.y, kChunkSize),
          floorDiv(index.z, kChunkSize)};
}

std::size_t OccupancyGrid3D::localBitIndex(const GridIndex3D index) noexcept {
  const auto local_x = static_cast<std::size_t>(positiveModulo(index.x, kChunkSize));
  const auto local_y = static_cast<std::size_t>(positiveModulo(index.y, kChunkSize));
  const auto local_z = static_cast<std::size_t>(positiveModulo(index.z, kChunkSize));
  return (local_z * static_cast<std::size_t>(kChunkSize) + local_y) *
             static_cast<std::size_t>(kChunkSize) +
         local_x;
}

bool OccupancyGrid3D::isOccupied(const GridIndex3D index) const noexcept {
  if (!contains(index)) {
    return false;
  }
  const auto chunk = chunks_.find(chunkIndex(index));
  if (chunk == chunks_.end()) {
    return false;
  }
  const std::size_t bit = localBitIndex(index);
  return (chunk->second.at(bit / 64U) & (std::uint64_t{1U} << (bit % 64U))) != 0U;
}

void OccupancyGrid3D::setOccupied(const GridIndex3D index) {
  if (!contains(index)) {
    throw std::out_of_range{"Occupancy3D index outside grid"};
  }
  Chunk& chunk = chunks_[chunkIndex(index)];
  const std::size_t bit = localBitIndex(index);
  const std::uint64_t mask = std::uint64_t{1U} << (bit % 64U);
  std::uint64_t& word = chunk.at(bit / 64U);
  if ((word & mask) == 0U) {
    word |= mask;
    ++occupied_voxels_;
  }
}

} // namespace drone_city_nav
