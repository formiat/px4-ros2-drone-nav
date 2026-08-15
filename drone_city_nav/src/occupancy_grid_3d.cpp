#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace drone_city_nav {
namespace {

constexpr std::array<char, 8U> kMagic{'D', 'C', 'N', 'O', 'C', 'C', '3', 'D'};
constexpr std::uint32_t kVersion{4U};
constexpr std::uint32_t kMaximumRegionCount{10000U};
constexpr std::uint32_t kMaximumPortalCount{100000U};
constexpr std::uint32_t kMaximumTraversalEdgeCount{100000U};
constexpr std::uint32_t kMaximumGeometryPointCount{100000U};

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

template<typename Value>
void writeValue(std::ostream& stream, const Value& value, const char* description) {
  static_assert(std::is_trivially_copyable_v<Value>);
  // Binary world artifacts intentionally expose trivially-copyable storage.
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream.write(reinterpret_cast<const char*>(&value),
               static_cast<std::streamsize>(sizeof(Value)));
  if (!stream) {
    throw std::runtime_error{std::string{"failed to write Occupancy3D "} + description};
  }
}

[[nodiscard]] std::uint32_t checkedCount(const std::size_t count,
                                         const std::uint32_t maximum,
                                         const char* description) {
  if (count > maximum) {
    throw std::runtime_error{std::string{"too many Occupancy3D "} + description};
  }
  return static_cast<std::uint32_t>(count);
}

void writeString(std::ostream& stream, const std::string& value,
                 const char* description) {
  if (value.empty() || value.size() > std::numeric_limits<std::uint16_t>::max()) {
    throw std::runtime_error{std::string{"invalid Occupancy3D "} + description};
  }
  writeValue(stream, static_cast<std::uint16_t>(value.size()), description);
  stream.write(value.data(), static_cast<std::streamsize>(value.size()));
  if (!stream) {
    throw std::runtime_error{std::string{"failed to write Occupancy3D "} + description};
  }
}

void writePoint(std::ostream& stream, const Point3& point, const char* description) {
  writeValue(stream, static_cast<float>(point.x), description);
  writeValue(stream, static_cast<float>(point.y), description);
  writeValue(stream, static_cast<float>(point.z), description);
}

void writeVector(std::ostream& stream, const Vec3& vector, const char* description) {
  writeValue(stream, static_cast<float>(vector.x), description);
  writeValue(stream, static_cast<float>(vector.y), description);
  writeValue(stream, static_cast<float>(vector.z), description);
}

void writePoints(std::ostream& stream, const std::vector<Point3>& points,
                 const char* description) {
  writeValue(stream,
             checkedCount(points.size(), kMaximumGeometryPointCount, description),
             description);
  for (const Point3& point : points) {
    writePoint(stream, point, description);
  }
}

[[nodiscard]] std::string readString(std::istream& stream, const char* description) {
  const std::uint16_t size = readValue<std::uint16_t>(stream, description);
  if (size == 0U) {
    throw std::runtime_error{std::string{"empty Occupancy3D "} + description};
  }
  std::string value(size, '\0');
  stream.read(value.data(), static_cast<std::streamsize>(value.size()));
  if (!stream) {
    throw std::runtime_error{std::string{"truncated Occupancy3D "} + description};
  }
  return value;
}

[[nodiscard]] Point3 readPoint(std::istream& stream, const char* description) {
  const Point3 point{static_cast<double>(readValue<float>(stream, description)),
                     static_cast<double>(readValue<float>(stream, description)),
                     static_cast<double>(readValue<float>(stream, description))};
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    throw std::runtime_error{std::string{"non-finite Occupancy3D "} + description};
  }
  return point;
}

[[nodiscard]] Vec3 readVector(std::istream& stream, const char* description) {
  const Vec3 vector{static_cast<double>(readValue<float>(stream, description)),
                    static_cast<double>(readValue<float>(stream, description)),
                    static_cast<double>(readValue<float>(stream, description))};
  if (!std::isfinite(vector.x) || !std::isfinite(vector.y) ||
      !std::isfinite(vector.z)) {
    throw std::runtime_error{std::string{"non-finite Occupancy3D "} + description};
  }
  return vector;
}

[[nodiscard]] std::vector<Point3> readPoints(std::istream& stream,
                                             const char* count_description,
                                             const char* point_description,
                                             const std::uint32_t minimum_count) {
  const std::uint32_t point_count = readValue<std::uint32_t>(stream, count_description);
  if (point_count < minimum_count || point_count > kMaximumGeometryPointCount) {
    throw std::runtime_error{std::string{"invalid Occupancy3D "} + count_description};
  }
  std::vector<Point3> points;
  points.reserve(point_count);
  for (std::uint32_t point_number = 0U; point_number < point_count; ++point_number) {
    points.push_back(readPoint(stream, point_description));
  }
  return points;
}

[[nodiscard]] DerivedPortalGraph readPortalGraph(std::istream& stream,
                                                 const GridBounds3D& bounds) {
  DerivedPortalGraph graph;
  const std::uint32_t region_count =
      readValue<std::uint32_t>(stream, "portal graph region count");
  if (region_count > kMaximumRegionCount) {
    throw std::runtime_error{
        "Occupancy3D portal graph region count exceeds supported range"};
  }
  std::set<FreeSpaceRegionId> region_ids;
  std::map<PassagePortalId, FreeSpaceRegionId> portal_regions;
  std::map<PassagePortalId, Point3> portal_centers;
  graph.regions.reserve(region_count);
  for (std::uint32_t region_number = 0U; region_number < region_count;
       ++region_number) {
    PassageRegion region{.id =
                             FreeSpaceRegionId{readString(stream, "passage region id")},
                         .portal_ids = {}};
    if (!region_ids.insert(region.id).second) {
      throw std::runtime_error{"duplicate Occupancy3D passage region id"};
    }
    const std::uint32_t portal_count =
        readValue<std::uint32_t>(stream, "passage region portal count");
    if (portal_count < 2U || portal_count > kMaximumPortalCount) {
      throw std::runtime_error{"invalid Occupancy3D passage region portal count"};
    }
    region.portal_ids.reserve(portal_count);
    for (std::uint32_t portal_number = 0U; portal_number < portal_count;
         ++portal_number) {
      PassagePortal portal{
          .id = PassagePortalId{readString(stream, "portal id")},
          .region_id = region.id,
          .center = readPoint(stream, "portal center"),
          .outward_normal = readVector(stream, "portal outward normal"),
          .opening_polygon = readPoints(stream, "portal polygon point count",
                                        "portal polygon point", 3U),
      };
      if (!portal_regions.emplace(portal.id, region.id).second) {
        throw std::runtime_error{"duplicate Occupancy3D portal id"};
      }
      const double normal_length =
          std::sqrt(portal.outward_normal.x * portal.outward_normal.x +
                    portal.outward_normal.y * portal.outward_normal.y +
                    portal.outward_normal.z * portal.outward_normal.z);
      if (std::abs(normal_length - 1.0) > 1.0e-5) {
        throw std::runtime_error{"invalid Occupancy3D portal normal"};
      }
      portal_centers.emplace(portal.id, portal.center);
      region.portal_ids.push_back(portal.id);
      graph.portals.push_back(std::move(portal));
    }
    graph.regions.push_back(std::move(region));
  }

  const std::uint32_t edge_count =
      readValue<std::uint32_t>(stream, "portal traversal edge count");
  if (edge_count > kMaximumTraversalEdgeCount) {
    throw std::runtime_error{"Occupancy3D portal edge count exceeds supported range"};
  }
  std::set<PassageTraversalId> edge_ids;
  graph.traversal_edges.reserve(edge_count);
  for (std::uint32_t edge_number = 0U; edge_number < edge_count; ++edge_number) {
    PassageTraversalEdge edge;
    edge.id = PassageTraversalId{readString(stream, "portal traversal edge id")};
    edge.region_id =
        FreeSpaceRegionId{readString(stream, "portal traversal region id")};
    edge.entry_portal_id =
        PassagePortalId{readString(stream, "portal traversal entry id")};
    edge.exit_portal_id =
        PassagePortalId{readString(stream, "portal traversal exit id")};
    const std::vector<Point3> points = readPoints(
        stream, "portal traversal point count", "portal traversal point", 2U);
    edge.min_z_m = static_cast<double>(readValue<float>(stream, "portal minimum z"));
    edge.max_z_m = static_cast<double>(readValue<float>(stream, "portal maximum z"));
    edge.width_m = static_cast<double>(readValue<float>(stream, "portal width"));
    edge.height_m = static_cast<double>(readValue<float>(stream, "portal height"));
    edge.minimum_clearance_m =
        static_cast<double>(readValue<float>(stream, "portal minimum clearance"));
    edge.speed_limit_mps =
        static_cast<double>(readValue<float>(stream, "portal speed limit"));
    const bool portal_geometry_matches =
        portal_centers.contains(edge.entry_portal_id) &&
        portal_centers.contains(edge.exit_portal_id) &&
        distance3D(points.front(), portal_centers.at(edge.entry_portal_id)) <= 1.0e-5 &&
        distance3D(points.back(), portal_centers.at(edge.exit_portal_id)) <= 1.0e-5;
    const bool valid =
        edge_ids.insert(edge.id).second && region_ids.contains(edge.region_id) &&
        portal_regions.contains(edge.entry_portal_id) &&
        portal_regions.contains(edge.exit_portal_id) &&
        portal_regions.at(edge.entry_portal_id) == edge.region_id &&
        portal_regions.at(edge.exit_portal_id) == edge.region_id &&
        edge.entry_portal_id != edge.exit_portal_id && portal_geometry_matches &&
        std::isfinite(edge.min_z_m) && std::isfinite(edge.max_z_m) &&
        edge.max_z_m > edge.min_z_m && edge.width_m > 0.0 && edge.height_m > 0.0 &&
        edge.minimum_clearance_m > 0.0 && edge.speed_limit_mps > 0.0;
    if (!valid) {
      throw std::runtime_error{"invalid Occupancy3D portal traversal edge"};
    }
    edge.centerline = sampleRoute3D(points, bounds.resolution_m, edge.speed_limit_mps);
    edge.entry = edge.centerline.front().position;
    edge.exit = edge.centerline.back().position;
    graph.traversal_edges.push_back(std::move(edge));
  }
  return graph;
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
  grid.portal_graph_ = readPortalGraph(stream, bounds);
  if (stream.peek() != std::char_traits<char>::eof()) {
    throw std::runtime_error{"trailing data in Occupancy3D artifact"};
  }
  return grid;
}

void OccupancyGrid3D::write(const std::filesystem::path& path) const {
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream{path, std::ios::binary};
  if (!stream) {
    throw std::runtime_error{"failed to create Occupancy3D: " + path.string()};
  }

  stream.write(kMagic.data(), static_cast<std::streamsize>(kMagic.size()));
  writeValue(stream, kVersion, "version");
  writeValue(stream, static_cast<std::uint32_t>(kChunkSize), "chunk size");
  writeValue(stream, static_cast<float>(bounds_.resolution_m), "resolution");
  writeValue(stream, static_cast<float>(bounds_.origin_x), "origin x");
  writeValue(stream, static_cast<float>(bounds_.origin_y), "origin y");
  writeValue(stream, static_cast<float>(bounds_.origin_z), "origin z");
  writeValue(stream, static_cast<std::uint32_t>(bounds_.width_cells), "width");
  writeValue(stream, static_cast<std::uint32_t>(bounds_.height_cells), "height");
  writeValue(stream, static_cast<std::uint32_t>(bounds_.depth_cells), "depth");
  writeValue(stream, fingerprint_, "fingerprint");
  writeValue(
      stream,
      checkedCount(chunks_.size(), std::numeric_limits<std::uint32_t>::max(), "chunks"),
      "chunk count");

  std::vector<std::pair<OccupancyChunkIndex3D, const Chunk*>> sorted_chunks;
  sorted_chunks.reserve(chunks_.size());
  for (const auto& [index, chunk] : chunks_) {
    sorted_chunks.emplace_back(index, &chunk);
  }
  std::ranges::sort(sorted_chunks, {}, [](const auto& item) {
    return std::tuple{item.first.x, item.first.y, item.first.z};
  });
  for (const auto& [index, chunk] : sorted_chunks) {
    writeValue(stream, static_cast<std::int32_t>(index.x), "chunk x");
    writeValue(stream, static_cast<std::int32_t>(index.y), "chunk y");
    writeValue(stream, static_cast<std::int32_t>(index.z), "chunk z");
    for (const std::uint64_t word : *chunk) {
      writeValue(stream, word, "chunk word");
    }
  }

  writeValue(stream,
             checkedCount(portal_graph_.regions.size(), kMaximumRegionCount,
                          "portal graph regions"),
             "portal graph region count");
  for (const PassageRegion& region : portal_graph_.regions) {
    writeString(stream, region.id.value(), "passage region id");
    writeValue(stream,
               checkedCount(region.portal_ids.size(), kMaximumPortalCount,
                            "passage region portals"),
               "passage region portal count");
    for (const PassagePortalId& portal_id : region.portal_ids) {
      const auto portal =
          std::ranges::find(portal_graph_.portals, portal_id, &PassagePortal::id);
      if (portal == portal_graph_.portals.end() || portal->region_id != region.id) {
        throw std::runtime_error{"invalid portal graph while writing Occupancy3D"};
      }
      writeString(stream, portal->id.value(), "portal id");
      writePoint(stream, portal->center, "portal center");
      writeVector(stream, portal->outward_normal, "portal outward normal");
      writePoints(stream, portal->opening_polygon, "portal polygon points");
    }
  }
  writeValue(stream,
             checkedCount(portal_graph_.traversal_edges.size(),
                          kMaximumTraversalEdgeCount, "portal traversal edges"),
             "portal traversal edge count");
  for (const PassageTraversalEdge& edge : portal_graph_.traversal_edges) {
    writeString(stream, edge.id.value(), "portal traversal edge id");
    writeString(stream, edge.region_id.value(), "portal traversal region id");
    writeString(stream, edge.entry_portal_id.value(), "portal traversal entry id");
    writeString(stream, edge.exit_portal_id.value(), "portal traversal exit id");
    std::vector<Point3> centerline;
    centerline.reserve(edge.centerline.size());
    std::ranges::transform(edge.centerline, std::back_inserter(centerline),
                           [](const RouteSample3D& sample) { return sample.position; });
    writePoints(stream, centerline, "portal traversal points");
    writeValue(stream, static_cast<float>(edge.min_z_m), "portal minimum z");
    writeValue(stream, static_cast<float>(edge.max_z_m), "portal maximum z");
    writeValue(stream, static_cast<float>(edge.width_m), "portal width");
    writeValue(stream, static_cast<float>(edge.height_m), "portal height");
    writeValue(stream, static_cast<float>(edge.minimum_clearance_m),
               "portal minimum clearance");
    writeValue(stream, static_cast<float>(edge.speed_limit_mps), "portal speed limit");
  }
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

const DerivedPortalGraph& OccupancyGrid3D::portalGraph() const noexcept {
  return portal_graph_;
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
