#include "drone_city_nav/free_space_topology_3d.hpp"

#include "drone_city_nav/free_space_topology_format_limits.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <ranges>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr std::array<char, 8U> kMagic{'D', 'C', 'N', 'F', 'T', 'O', 'P', '3'};
constexpr std::uint32_t kVersion{2U};
constexpr std::uint32_t kLegacyVersion{1U};
constexpr std::uint32_t kMaximumRegionCount{
    FreeSpaceTopologyFormatLimits::maximum_region_count};
constexpr std::uint32_t kMaximumPortalCount{
    FreeSpaceTopologyFormatLimits::maximum_portal_count};
constexpr std::uint32_t kMaximumSegmentCount{
    FreeSpaceTopologyFormatLimits::maximum_segment_count};
constexpr std::uint32_t kMaximumTraversalEdgeCount{
    FreeSpaceTopologyFormatLimits::maximum_traversal_edge_count};
constexpr std::uint32_t kMaximumGeometryPointCount{
    FreeSpaceTopologyFormatLimits::maximum_geometry_point_count};
constexpr std::size_t kMaximumTotalGeometryPointCount{
    FreeSpaceTopologyFormatLimits::maximum_total_geometry_point_count};

template<typename Value>
[[nodiscard]] Value readValue(std::istream& stream, const char* description) {
  static_assert(std::is_trivially_copyable_v<Value>);
  Value value{};
  // Binary world artifacts intentionally expose trivially-copyable storage.
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  stream.read(reinterpret_cast<char*>(&value),
              static_cast<std::streamsize>(sizeof(Value)));
  if (!stream) {
    throw std::runtime_error{
        std::string{"truncated FreeSpaceTopology3D while reading "} + description};
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
    throw std::runtime_error{std::string{"failed to write FreeSpaceTopology3D "} +
                             description};
  }
}

[[nodiscard]] std::uint32_t checkedCount(const std::size_t count,
                                         const std::uint32_t maximum,
                                         const char* description) {
  if (count > maximum) {
    throw std::runtime_error{std::string{"too many FreeSpaceTopology3D "} +
                             description};
  }
  return static_cast<std::uint32_t>(count);
}

void writeString(std::ostream& stream, const std::string& value,
                 const char* description) {
  if (value.empty() || value.size() > std::numeric_limits<std::uint16_t>::max()) {
    throw std::runtime_error{std::string{"invalid FreeSpaceTopology3D "} + description};
  }
  writeValue(stream, static_cast<std::uint16_t>(value.size()), description);
  stream.write(value.data(), static_cast<std::streamsize>(value.size()));
  if (!stream) {
    throw std::runtime_error{std::string{"failed to write FreeSpaceTopology3D "} +
                             description};
  }
}

[[nodiscard]] std::string readString(std::istream& stream, const char* description) {
  const std::uint16_t size = readValue<std::uint16_t>(stream, description);
  if (size == 0U) {
    throw std::runtime_error{std::string{"empty FreeSpaceTopology3D "} + description};
  }
  std::string value(size, '\0');
  stream.read(value.data(), static_cast<std::streamsize>(value.size()));
  if (!stream) {
    throw std::runtime_error{
        std::string{"truncated FreeSpaceTopology3D while reading "} + description};
  }
  return value;
}

[[nodiscard]] bool finite(const Point3& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finite(const Vec3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
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

[[nodiscard]] Point3 readPoint(std::istream& stream, const char* description) {
  const Point3 point{static_cast<double>(readValue<float>(stream, description)),
                     static_cast<double>(readValue<float>(stream, description)),
                     static_cast<double>(readValue<float>(stream, description))};
  if (!finite(point)) {
    throw std::runtime_error{std::string{"non-finite FreeSpaceTopology3D "} +
                             description};
  }
  return point;
}

[[nodiscard]] Vec3 readVector(std::istream& stream, const char* description) {
  const Vec3 vector{static_cast<double>(readValue<float>(stream, description)),
                    static_cast<double>(readValue<float>(stream, description)),
                    static_cast<double>(readValue<float>(stream, description))};
  if (!finite(vector)) {
    throw std::runtime_error{std::string{"non-finite FreeSpaceTopology3D "} +
                             description};
  }
  return vector;
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

[[nodiscard]] std::vector<Point3> readPoints(std::istream& stream,
                                             const char* count_description,
                                             const char* point_description,
                                             const std::uint32_t minimum_count) {
  const std::uint32_t point_count = readValue<std::uint32_t>(stream, count_description);
  if (point_count < minimum_count || point_count > kMaximumGeometryPointCount) {
    throw std::runtime_error{std::string{"invalid FreeSpaceTopology3D "} +
                             count_description};
  }
  std::vector<Point3> points;
  points.reserve(point_count);
  for (std::uint32_t point_number = 0U; point_number < point_count; ++point_number) {
    points.push_back(readPoint(stream, point_description));
  }
  return points;
}

template<typename Id>
void writeIds(std::ostream& stream, const std::vector<Id>& ids,
              const std::uint32_t maximum_count, const char* count_description,
              const char* id_description) {
  writeValue(stream, checkedCount(ids.size(), maximum_count, count_description),
             count_description);
  for (const Id& id : ids) {
    writeString(stream, id.value(), id_description);
  }
}

template<typename Id>
[[nodiscard]] std::vector<Id>
readIds(std::istream& stream, const std::uint32_t maximum_count,
        const char* count_description, const char* id_description) {
  const std::uint32_t count = readValue<std::uint32_t>(stream, count_description);
  if (count > maximum_count) {
    throw std::runtime_error{std::string{"invalid FreeSpaceTopology3D "} +
                             count_description};
  }
  std::vector<Id> ids;
  ids.reserve(count);
  for (std::uint32_t index = 0U; index < count; ++index) {
    ids.emplace_back(readString(stream, id_description));
  }
  return ids;
}

void writeGridCells(std::ostream& stream, const std::vector<GridIndex3D>& cells,
                    const char* description) {
  writeValue(stream,
             checkedCount(cells.size(), kMaximumGeometryPointCount, description),
             description);
  for (const GridIndex3D cell : cells) {
    writeValue(stream, static_cast<std::int32_t>(cell.x), description);
    writeValue(stream, static_cast<std::int32_t>(cell.y), description);
    writeValue(stream, static_cast<std::int32_t>(cell.z), description);
  }
}

[[nodiscard]] std::vector<GridIndex3D> readGridCells(std::istream& stream,
                                                     const char* description) {
  const std::uint32_t count = readValue<std::uint32_t>(stream, description);
  if (count > kMaximumGeometryPointCount) {
    throw std::runtime_error{std::string{"invalid FreeSpaceTopology3D "} + description};
  }
  std::vector<GridIndex3D> cells;
  cells.reserve(count);
  for (std::uint32_t index = 0U; index < count; ++index) {
    cells.push_back(GridIndex3D{
        static_cast<int>(readValue<std::int32_t>(stream, description)),
        static_cast<int>(readValue<std::int32_t>(stream, description)),
        static_cast<int>(readValue<std::int32_t>(stream, description)),
    });
  }
  return cells;
}

[[nodiscard]] std::vector<Point3>
centerlinePoints(const std::vector<RouteSample3D>& centerline) {
  std::vector<Point3> points;
  points.reserve(centerline.size());
  std::ranges::transform(centerline, std::back_inserter(points),
                         [](const RouteSample3D& sample) { return sample.position; });
  return points;
}

[[nodiscard]] GridBounds3D readBounds(std::istream& stream) {
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
    throw std::runtime_error{"FreeSpaceTopology3D dimensions exceed supported range"};
  }
  bounds.width_cells = static_cast<int>(width);
  bounds.height_cells = static_cast<int>(height);
  bounds.depth_cells = static_cast<int>(depth);
  return bounds;
}

void writeBounds(std::ostream& stream, const GridBounds3D& bounds) {
  writeValue(stream, static_cast<float>(bounds.resolution_m), "resolution");
  writeValue(stream, static_cast<float>(bounds.origin_x), "origin x");
  writeValue(stream, static_cast<float>(bounds.origin_y), "origin y");
  writeValue(stream, static_cast<float>(bounds.origin_z), "origin z");
  writeValue(stream, static_cast<std::uint32_t>(bounds.width_cells), "width");
  writeValue(stream, static_cast<std::uint32_t>(bounds.height_cells), "height");
  writeValue(stream, static_cast<std::uint32_t>(bounds.depth_cells), "depth");
}

[[nodiscard]] std::vector<RouteSample3D>
restoreCenterline(const std::span<const Point3> points,
                  const double reference_speed_mps) {
  std::vector<RouteSample3D> result;
  result.reserve(points.size());
  result.push_back(RouteSample3D{.position = points.front(),
                                 .reference_speed_mps = reference_speed_mps});
  double station_m = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double segment_length_m = distance3D(points[index - 1U], points[index]);
    if (!std::isfinite(segment_length_m) || !(segment_length_m > 1.0e-9)) {
      throw std::runtime_error{
          "invalid FreeSpaceTopology3D consecutive traversal points"};
    }
    station_m += segment_length_m;
    const Vec3 tangent{
        (points[index].x - points[index - 1U].x) / segment_length_m,
        (points[index].y - points[index - 1U].y) / segment_length_m,
        (points[index].z - points[index - 1U].z) / segment_length_m,
    };
    result.push_back(RouteSample3D{
        .position = points[index],
        .tangent = tangent,
        .station_m = station_m,
        .reference_speed_mps = reference_speed_mps,
    });
  }
  result.front().tangent = result[1U].tangent;
  return result;
}

struct TopologyData {
  std::vector<FreeSpaceRegion> regions;
  std::vector<PassagePortal> portals;
  std::vector<PassageSegment> segments;
  std::vector<PassageTraversalEdge> traversal_edges;
};

[[nodiscard]] PassageTraversalEdge readLegacyTraversal(std::istream& stream,
                                                       std::size_t& geometry_points) {
  PassageTraversalEdge edge;
  edge.id = PassageTraversalId{readString(stream, "passage traversal id")};
  edge.region_id = FreeSpaceRegionId{readString(stream, "passage traversal region id")};
  edge.entry_portal_id =
      PassagePortalId{readString(stream, "passage traversal entry portal id")};
  edge.exit_portal_id =
      PassagePortalId{readString(stream, "passage traversal exit portal id")};
  const std::vector<Point3> centerline = readPoints(
      stream, "passage traversal point count", "passage traversal point", 2U);
  geometry_points += centerline.size();
  if (geometry_points > kMaximumTotalGeometryPointCount) {
    throw std::runtime_error{"FreeSpaceTopology3D geometry exceeds supported range"};
  }
  edge.min_z_m = static_cast<double>(readValue<float>(stream, "passage minimum z"));
  edge.max_z_m = static_cast<double>(readValue<float>(stream, "passage maximum z"));
  edge.width_m = static_cast<double>(readValue<float>(stream, "passage width"));
  edge.height_m = static_cast<double>(readValue<float>(stream, "passage height"));
  edge.minimum_clearance_m =
      static_cast<double>(readValue<float>(stream, "passage minimum clearance"));
  edge.speed_limit_mps =
      static_cast<double>(readValue<float>(stream, "passage speed limit"));
  edge.centerline = restoreCenterline(centerline, edge.speed_limit_mps);
  edge.entry = edge.centerline.front().position;
  edge.exit = edge.centerline.back().position;
  return edge;
}

[[nodiscard]] TopologyData readLegacyData(std::istream& stream) {
  TopologyData data;
  std::size_t geometry_points{0U};
  const std::uint32_t region_count = readValue<std::uint32_t>(stream, "region count");
  if (region_count > kMaximumRegionCount) {
    throw std::runtime_error{
        "FreeSpaceTopology3D region count exceeds supported range"};
  }
  data.regions.reserve(region_count);
  for (std::uint32_t region_number = 0U; region_number < region_count;
       ++region_number) {
    FreeSpaceRegion region{
        .id = FreeSpaceRegionId{readString(stream, "free-space region id")},
        .representative = {},
        .maximum_clearance_m = 0.0,
        .portal_ids = {},
    };
    const std::uint32_t portal_count =
        readValue<std::uint32_t>(stream, "region portal count");
    if (portal_count < 2U || portal_count > kMaximumPortalCount ||
        portal_count > kMaximumPortalCount - data.portals.size()) {
      throw std::runtime_error{"invalid FreeSpaceTopology3D region portal count"};
    }
    region.portal_ids.reserve(portal_count);
    data.portals.reserve(data.portals.size() + portal_count);
    for (std::uint32_t portal_number = 0U; portal_number < portal_count;
         ++portal_number) {
      PassagePortal portal{
          .id = PassagePortalId{readString(stream, "passage portal id")},
          .region_id = region.id,
          .center = readPoint(stream, "passage portal center"),
          .outward_normal = readVector(stream, "passage portal outward normal"),
          .opening_polygon = readPoints(stream, "passage portal polygon point count",
                                        "passage portal polygon point", 3U),
          .surface_voxels = {},
          .traversable_anchors = {},
          .local_u_axis = {},
          .local_v_axis = {},
          .minimum_clearance_m = 0.0,
          .mean_clearance_m = 0.0,
          .maximum_clearance_m = 0.0,
      };
      geometry_points += portal.opening_polygon.size();
      if (geometry_points > kMaximumTotalGeometryPointCount) {
        throw std::runtime_error{
            "FreeSpaceTopology3D geometry exceeds supported range"};
      }
      region.portal_ids.push_back(portal.id);
      data.portals.push_back(std::move(portal));
    }
    data.regions.push_back(std::move(region));
  }
  const std::uint32_t edge_count =
      readValue<std::uint32_t>(stream, "passage traversal count");
  if (edge_count > kMaximumTraversalEdgeCount) {
    throw std::runtime_error{
        "FreeSpaceTopology3D traversal count exceeds supported range"};
  }
  data.traversal_edges.reserve(edge_count);
  for (std::uint32_t edge_number = 0U; edge_number < edge_count; ++edge_number) {
    data.traversal_edges.push_back(readLegacyTraversal(stream, geometry_points));
  }
  return data;
}

[[nodiscard]] TopologyData readVersion2Data(std::istream& stream) {
  TopologyData data;
  std::size_t geometry_points{0U};
  const std::uint32_t region_count = readValue<std::uint32_t>(stream, "region count");
  if (region_count > kMaximumRegionCount) {
    throw std::runtime_error{
        "FreeSpaceTopology3D region count exceeds supported range"};
  }
  data.regions.reserve(region_count);
  for (std::uint32_t index = 0U; index < region_count; ++index) {
    data.regions.push_back(FreeSpaceRegion{
        .id = FreeSpaceRegionId{readString(stream, "free-space region id")},
        .representative = readPoint(stream, "free-space region representative"),
        .maximum_clearance_m = static_cast<double>(
            readValue<float>(stream, "free-space region maximum clearance")),
        .portal_ids = readIds<PassagePortalId>(stream, kMaximumPortalCount,
                                               "free-space region portal count",
                                               "free-space region portal id"),
    });
  }

  const std::uint32_t portal_count = readValue<std::uint32_t>(stream, "portal count");
  if (portal_count > kMaximumPortalCount) {
    throw std::runtime_error{
        "FreeSpaceTopology3D portal count exceeds supported range"};
  }
  data.portals.reserve(portal_count);
  for (std::uint32_t index = 0U; index < portal_count; ++index) {
    PassagePortal portal{
        .id = PassagePortalId{readString(stream, "passage portal id")},
        .region_id = FreeSpaceRegionId{readString(stream, "passage portal region id")},
        .center = readPoint(stream, "passage portal center"),
        .outward_normal = readVector(stream, "passage portal outward normal"),
        .opening_polygon = readPoints(stream, "passage portal polygon point count",
                                      "passage portal polygon point", 3U),
        .surface_voxels = readGridCells(stream, "passage portal surface voxels"),
        .traversable_anchors = readPoints(stream, "passage portal anchor count",
                                          "passage portal anchor", 0U),
        .local_u_axis = readVector(stream, "passage portal local u axis"),
        .local_v_axis = readVector(stream, "passage portal local v axis"),
        .minimum_clearance_m = static_cast<double>(
            readValue<float>(stream, "passage portal minimum clearance")),
        .mean_clearance_m = static_cast<double>(
            readValue<float>(stream, "passage portal mean clearance")),
        .maximum_clearance_m = static_cast<double>(
            readValue<float>(stream, "passage portal maximum clearance")),
    };
    geometry_points += portal.opening_polygon.size() + portal.surface_voxels.size() +
                       portal.traversable_anchors.size();
    if (geometry_points > kMaximumTotalGeometryPointCount) {
      throw std::runtime_error{"FreeSpaceTopology3D geometry exceeds supported range"};
    }
    data.portals.push_back(std::move(portal));
  }

  const std::uint32_t segment_count =
      readValue<std::uint32_t>(stream, "passage segment count");
  if (segment_count > kMaximumSegmentCount) {
    throw std::runtime_error{
        "FreeSpaceTopology3D segment count exceeds supported range"};
  }
  data.segments.reserve(segment_count);
  for (std::uint32_t index = 0U; index < segment_count; ++index) {
    PassageSegment segment;
    segment.id = PassageSegmentId{readString(stream, "passage segment id")};
    const std::vector<Point3> points =
        readPoints(stream, "passage segment point count", "passage segment point", 2U);
    segment.endpoint_portal_ids =
        readIds<PassagePortalId>(stream, 2U, "passage segment endpoint portal count",
                                 "passage segment endpoint portal id");
    segment.first_endpoint_neighbors = readIds<PassageSegmentId>(
        stream, kMaximumSegmentCount, "passage segment first neighbor count",
        "passage segment first neighbor id");
    segment.second_endpoint_neighbors = readIds<PassageSegmentId>(
        stream, kMaximumSegmentCount, "passage segment second neighbor count",
        "passage segment second neighbor id");
    segment.minimum_clearance_m = static_cast<double>(
        readValue<float>(stream, "passage segment minimum clearance"));
    segment.speed_limit_mps =
        static_cast<double>(readValue<float>(stream, "passage segment speed limit"));
    segment.centerline = restoreCenterline(points, segment.speed_limit_mps);
    geometry_points += points.size();
    if (geometry_points > kMaximumTotalGeometryPointCount) {
      throw std::runtime_error{"FreeSpaceTopology3D geometry exceeds supported range"};
    }
    data.segments.push_back(std::move(segment));
  }

  const std::uint32_t edge_count =
      readValue<std::uint32_t>(stream, "legacy passage traversal count");
  if (edge_count > kMaximumTraversalEdgeCount) {
    throw std::runtime_error{
        "FreeSpaceTopology3D traversal count exceeds supported range"};
  }
  data.traversal_edges.reserve(edge_count);
  for (std::uint32_t index = 0U; index < edge_count; ++index) {
    data.traversal_edges.push_back(readLegacyTraversal(stream, geometry_points));
  }
  return data;
}

void writeLegacyTraversal(std::ostream& stream, const PassageTraversalEdge& edge) {
  writeString(stream, edge.id.value(), "passage traversal id");
  writeString(stream, edge.region_id.value(), "passage traversal region id");
  writeString(stream, edge.entry_portal_id.value(),
              "passage traversal entry portal id");
  writeString(stream, edge.exit_portal_id.value(), "passage traversal exit portal id");
  writePoints(stream, centerlinePoints(edge.centerline), "passage traversal points");
  writeValue(stream, static_cast<float>(edge.min_z_m), "passage minimum z");
  writeValue(stream, static_cast<float>(edge.max_z_m), "passage maximum z");
  writeValue(stream, static_cast<float>(edge.width_m), "passage width");
  writeValue(stream, static_cast<float>(edge.height_m), "passage height");
  writeValue(stream, static_cast<float>(edge.minimum_clearance_m),
             "passage minimum clearance");
  writeValue(stream, static_cast<float>(edge.speed_limit_mps), "passage speed limit");
}

} // namespace

FreeSpaceTopology3D::FreeSpaceTopology3D(
    const std::uint64_t occupancy_fingerprint, const GridBounds3D& occupancy_bounds,
    std::vector<FreeSpaceRegion> regions, std::vector<PassagePortal> portals,
    std::vector<PassageTraversalEdge> traversal_edges)
    : FreeSpaceTopology3D{occupancy_fingerprint,
                          occupancy_bounds,
                          std::move(regions),
                          std::move(portals),
                          {},
                          std::move(traversal_edges)} {
}

FreeSpaceTopology3D::FreeSpaceTopology3D(
    const std::uint64_t occupancy_fingerprint, const GridBounds3D& occupancy_bounds,
    std::vector<FreeSpaceRegion> regions, std::vector<PassagePortal> portals,
    std::vector<PassageSegment> segments,
    std::vector<PassageTraversalEdge> legacy_traversal_edges)
    : occupancy_fingerprint_{occupancy_fingerprint},
      occupancy_bounds_{occupancy_bounds},
      regions_{std::move(regions)},
      portals_{std::move(portals)},
      segments_{std::move(segments)},
      traversal_edges_{std::move(legacy_traversal_edges)} {
  validate();
}

FreeSpaceTopology3D FreeSpaceTopology3D::load(const std::filesystem::path& path) {
  std::ifstream stream{path, std::ios::binary};
  if (!stream) {
    throw std::runtime_error{"failed to open FreeSpaceTopology3D: " + path.string()};
  }
  std::array<char, 8U> magic{};
  stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  if (!stream || magic != kMagic) {
    throw std::runtime_error{"invalid FreeSpaceTopology3D magic: " + path.string()};
  }
  const std::uint32_t version = readValue<std::uint32_t>(stream, "version");
  if (version != kVersion && version != kLegacyVersion) {
    throw std::runtime_error{"unsupported FreeSpaceTopology3D version"};
  }
  const std::uint64_t occupancy_fingerprint =
      readValue<std::uint64_t>(stream, "occupancy fingerprint");
  const GridBounds3D occupancy_bounds = readBounds(stream);

  TopologyData data =
      version == kLegacyVersion ? readLegacyData(stream) : readVersion2Data(stream);
  if (stream.peek() != std::char_traits<char>::eof()) {
    throw std::runtime_error{"trailing data in FreeSpaceTopology3D artifact"};
  }
  return FreeSpaceTopology3D{occupancy_fingerprint,    occupancy_bounds,
                             std::move(data.regions),  std::move(data.portals),
                             std::move(data.segments), std::move(data.traversal_edges)};
}

void FreeSpaceTopology3D::write(const std::filesystem::path& path) const {
  validate();
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  std::ofstream stream{path, std::ios::binary};
  if (!stream) {
    throw std::runtime_error{"failed to create FreeSpaceTopology3D: " + path.string()};
  }
  stream.write(kMagic.data(), static_cast<std::streamsize>(kMagic.size()));
  writeValue(stream, kVersion, "version");
  writeValue(stream, occupancy_fingerprint_, "occupancy fingerprint");
  writeBounds(stream, occupancy_bounds_);
  writeValue(stream, checkedCount(regions_.size(), kMaximumRegionCount, "regions"),
             "region count");
  for (const FreeSpaceRegion& region : regions_) {
    writeString(stream, region.id.value(), "free-space region id");
    writePoint(stream, region.representative, "free-space region representative");
    writeValue(stream, static_cast<float>(region.maximum_clearance_m),
               "free-space region maximum clearance");
    writeIds(stream, region.portal_ids, kMaximumPortalCount,
             "free-space region portal count", "free-space region portal id");
  }
  writeValue(stream, checkedCount(portals_.size(), kMaximumPortalCount, "portals"),
             "portal count");
  for (const PassagePortal& portal : portals_) {
    writeString(stream, portal.id.value(), "passage portal id");
    writeString(stream, portal.region_id.value(), "passage portal region id");
    writePoint(stream, portal.center, "passage portal center");
    writeVector(stream, portal.outward_normal, "passage portal outward normal");
    writePoints(stream, portal.opening_polygon, "passage portal polygon points");
    writeGridCells(stream, portal.surface_voxels, "passage portal surface voxels");
    writePoints(stream, portal.traversable_anchors, "passage portal anchors");
    writeVector(stream, portal.local_u_axis, "passage portal local u axis");
    writeVector(stream, portal.local_v_axis, "passage portal local v axis");
    writeValue(stream, static_cast<float>(portal.minimum_clearance_m),
               "passage portal minimum clearance");
    writeValue(stream, static_cast<float>(portal.mean_clearance_m),
               "passage portal mean clearance");
    writeValue(stream, static_cast<float>(portal.maximum_clearance_m),
               "passage portal maximum clearance");
  }
  writeValue(stream, checkedCount(segments_.size(), kMaximumSegmentCount, "segments"),
             "passage segment count");
  for (const PassageSegment& segment : segments_) {
    writeString(stream, segment.id.value(), "passage segment id");
    writePoints(stream, centerlinePoints(segment.centerline), "passage segment points");
    writeIds(stream, segment.endpoint_portal_ids, 2U,
             "passage segment endpoint portal count",
             "passage segment endpoint portal id");
    writeIds(stream, segment.first_endpoint_neighbors, kMaximumSegmentCount,
             "passage segment first neighbor count",
             "passage segment first neighbor id");
    writeIds(stream, segment.second_endpoint_neighbors, kMaximumSegmentCount,
             "passage segment second neighbor count",
             "passage segment second neighbor id");
    writeValue(stream, static_cast<float>(segment.minimum_clearance_m),
               "passage segment minimum clearance");
    writeValue(stream, static_cast<float>(segment.speed_limit_mps),
               "passage segment speed limit");
  }
  writeValue(stream,
             checkedCount(traversal_edges_.size(), kMaximumTraversalEdgeCount,
                          "passage traversals"),
             "passage traversal count");
  for (const PassageTraversalEdge& edge : traversal_edges_) {
    writeLegacyTraversal(stream, edge);
  }
}

std::uint64_t FreeSpaceTopology3D::occupancyFingerprint() const noexcept {
  return occupancy_fingerprint_;
}

const GridBounds3D& FreeSpaceTopology3D::occupancyBounds() const noexcept {
  return occupancy_bounds_;
}

bool FreeSpaceTopology3D::compatibleWith(
    const OccupancyGrid3D& occupancy) const noexcept {
  return occupancy_fingerprint_ == occupancy.fingerprint() &&
         occupancy_bounds_ == occupancy.bounds();
}

const std::vector<FreeSpaceRegion>& FreeSpaceTopology3D::regions() const noexcept {
  return regions_;
}

const std::vector<PassagePortal>& FreeSpaceTopology3D::portals() const noexcept {
  return portals_;
}

const std::vector<PassageSegment>& FreeSpaceTopology3D::segments() const noexcept {
  return segments_;
}

const std::vector<PassageTraversalEdge>&
FreeSpaceTopology3D::traversalEdges() const noexcept {
  return traversal_edges_;
}

void FreeSpaceTopology3D::validate() const {
  if (!(occupancy_bounds_.resolution_m > 0.0) ||
      !std::isfinite(occupancy_bounds_.resolution_m) ||
      !std::isfinite(occupancy_bounds_.origin_x) ||
      !std::isfinite(occupancy_bounds_.origin_y) ||
      !std::isfinite(occupancy_bounds_.origin_z) ||
      occupancy_bounds_.width_cells <= 0 || occupancy_bounds_.height_cells <= 0 ||
      occupancy_bounds_.depth_cells <= 0 || regions_.size() > kMaximumRegionCount ||
      portals_.size() > kMaximumPortalCount ||
      segments_.size() > kMaximumSegmentCount ||
      traversal_edges_.size() > kMaximumTraversalEdgeCount) {
    throw std::invalid_argument{"invalid FreeSpaceTopology3D metadata"};
  }

  std::set<FreeSpaceRegionId> region_ids;
  std::map<PassagePortalId, FreeSpaceRegionId> portal_regions;
  std::map<PassagePortalId, Point3> portal_centers;
  const bool sparse_topology = !segments_.empty();
  for (const PassagePortal& portal : portals_) {
    const double normal_length =
        std::sqrt(portal.outward_normal.x * portal.outward_normal.x +
                  portal.outward_normal.y * portal.outward_normal.y +
                  portal.outward_normal.z * portal.outward_normal.z);
    const double u_length =
        std::hypot(std::hypot(portal.local_u_axis.x, portal.local_u_axis.y),
                   portal.local_u_axis.z);
    const double v_length =
        std::hypot(std::hypot(portal.local_v_axis.x, portal.local_v_axis.y),
                   portal.local_v_axis.z);
    const double normal_u_dot = portal.outward_normal.x * portal.local_u_axis.x +
                                portal.outward_normal.y * portal.local_u_axis.y +
                                portal.outward_normal.z * portal.local_u_axis.z;
    const double normal_v_dot = portal.outward_normal.x * portal.local_v_axis.x +
                                portal.outward_normal.y * portal.local_v_axis.y +
                                portal.outward_normal.z * portal.local_v_axis.z;
    const double uv_dot = portal.local_u_axis.x * portal.local_v_axis.x +
                          portal.local_u_axis.y * portal.local_v_axis.y +
                          portal.local_u_axis.z * portal.local_v_axis.z;
    const bool base_geometry_valid =
        finite(portal.center) && finite(portal.outward_normal) &&
        std::abs(normal_length - 1.0) <= 1.0e-5 &&
        portal.opening_polygon.size() >= 3U &&
        portal.opening_polygon.size() <= kMaximumGeometryPointCount &&
        std::ranges::all_of(portal.opening_polygon,
                            [](const Point3& point) { return finite(point); });
    const bool sparse_geometry_valid =
        !sparse_topology ||
        (!portal.surface_voxels.empty() &&
         portal.surface_voxels.size() <= kMaximumGeometryPointCount &&
         !portal.traversable_anchors.empty() &&
         portal.traversable_anchors.size() <= kMaximumGeometryPointCount &&
         std::ranges::all_of(portal.traversable_anchors,
                             [](const Point3& point) { return finite(point); }) &&
         finite(portal.local_u_axis) && finite(portal.local_v_axis) &&
         std::abs(u_length - 1.0) <= 1.0e-5 && std::abs(v_length - 1.0) <= 1.0e-5 &&
         std::abs(normal_u_dot) <= 1.0e-5 && std::abs(normal_v_dot) <= 1.0e-5 &&
         std::abs(uv_dot) <= 1.0e-5 && std::isfinite(portal.minimum_clearance_m) &&
         portal.minimum_clearance_m > 0.0 && std::isfinite(portal.mean_clearance_m) &&
         portal.mean_clearance_m >= portal.minimum_clearance_m &&
         std::isfinite(portal.maximum_clearance_m) &&
         portal.maximum_clearance_m >= portal.mean_clearance_m &&
         std::ranges::all_of(portal.surface_voxels, [this](const GridIndex3D cell) {
           return cell.x >= 0 && cell.y >= 0 && cell.z >= 0 &&
                  cell.x < occupancy_bounds_.width_cells &&
                  cell.y < occupancy_bounds_.height_cells &&
                  cell.z < occupancy_bounds_.depth_cells;
         }));
    if (portal.id.empty() || portal.region_id.empty() || !base_geometry_valid ||
        !sparse_geometry_valid ||
        !portal_regions.emplace(portal.id, portal.region_id).second) {
      throw std::invalid_argument{"invalid FreeSpaceTopology3D passage portal"};
    }
    portal_centers.emplace(portal.id, portal.center);
  }

  std::set<PassagePortalId> referenced_portals;
  for (const FreeSpaceRegion& region : regions_) {
    if (region.id.empty() || !region_ids.insert(region.id).second ||
        region.portal_ids.size() < (sparse_topology ? 1U : 2U) ||
        region.portal_ids.size() > kMaximumPortalCount) {
      throw std::invalid_argument{"invalid FreeSpaceTopology3D region"};
    }
    if (sparse_topology &&
        (!finite(region.representative) || !std::isfinite(region.maximum_clearance_m) ||
         !(region.maximum_clearance_m > 0.0))) {
      throw std::invalid_argument{"invalid FreeSpaceTopology3D sparse region"};
    }
    for (const PassagePortalId& portal_id : region.portal_ids) {
      if (!portal_regions.contains(portal_id) ||
          portal_regions.at(portal_id) != region.id ||
          !referenced_portals.insert(portal_id).second) {
        throw std::invalid_argument{
            "invalid FreeSpaceTopology3D region portal ownership"};
      }
    }
  }
  if (referenced_portals.size() != portals_.size()) {
    throw std::invalid_argument{"unowned FreeSpaceTopology3D passage portal"};
  }

  std::map<PassageSegmentId, const PassageSegment*> segment_by_id;
  std::set<PassagePortalId> segment_portals;
  for (const PassageSegment& segment : segments_) {
    const bool centerline_valid =
        segment.centerline.size() >= 2U &&
        segment.centerline.size() <= kMaximumGeometryPointCount &&
        std::ranges::all_of(segment.centerline, [](const RouteSample3D& sample) {
          return finite(sample.position);
        });
    const bool portals_valid =
        segment.endpoint_portal_ids.size() <= 2U &&
        std::ranges::all_of(segment.endpoint_portal_ids,
                            [&portal_regions](const PassagePortalId& portal_id) {
                              return portal_regions.contains(portal_id);
                            });
    const bool valid =
        !segment.id.empty() && segment_by_id.emplace(segment.id, &segment).second &&
        centerline_valid && portals_valid &&
        segment.first_endpoint_neighbors.size() <= kMaximumSegmentCount &&
        segment.second_endpoint_neighbors.size() <= kMaximumSegmentCount &&
        std::isfinite(segment.minimum_clearance_m) &&
        segment.minimum_clearance_m > 0.0 && std::isfinite(segment.speed_limit_mps) &&
        segment.speed_limit_mps > 0.0;
    if (!valid) {
      throw std::invalid_argument{"invalid FreeSpaceTopology3D passage segment"};
    }
    for (const PassagePortalId& portal_id : segment.endpoint_portal_ids) {
      const Point3& portal_center = portal_centers.at(portal_id);
      const bool endpoint_matches =
          distance3D(segment.centerline.front().position, portal_center) <= 1.0e-5 ||
          distance3D(segment.centerline.back().position, portal_center) <= 1.0e-5;
      if (!endpoint_matches) {
        throw std::invalid_argument{
            "invalid FreeSpaceTopology3D segment portal ownership"};
      }
      segment_portals.insert(portal_id);
    }
  }
  for (const PassageSegment& segment : segments_) {
    const auto neighbors_valid = [&](const std::vector<PassageSegmentId>& neighbors) {
      return std::ranges::all_of(neighbors, [&](const PassageSegmentId& neighbor_id) {
        if (neighbor_id == segment.id || !segment_by_id.contains(neighbor_id)) {
          return false;
        }
        const PassageSegment& neighbor = *segment_by_id.at(neighbor_id);
        const bool reciprocal =
            std::ranges::find(neighbor.first_endpoint_neighbors, segment.id) !=
                neighbor.first_endpoint_neighbors.end() ||
            std::ranges::find(neighbor.second_endpoint_neighbors, segment.id) !=
                neighbor.second_endpoint_neighbors.end();
        const bool endpoints_touch =
            distance3D(segment.centerline.front().position,
                       neighbor.centerline.front().position) <= 1.0e-5 ||
            distance3D(segment.centerline.front().position,
                       neighbor.centerline.back().position) <= 1.0e-5 ||
            distance3D(segment.centerline.back().position,
                       neighbor.centerline.front().position) <= 1.0e-5 ||
            distance3D(segment.centerline.back().position,
                       neighbor.centerline.back().position) <= 1.0e-5;
        return reciprocal && endpoints_touch;
      });
    };
    if (!neighbors_valid(segment.first_endpoint_neighbors) ||
        !neighbors_valid(segment.second_endpoint_neighbors)) {
      throw std::invalid_argument{"invalid FreeSpaceTopology3D segment adjacency"};
    }
  }
  if (sparse_topology && segment_portals.size() != portals_.size()) {
    throw std::invalid_argument{"unconnected FreeSpaceTopology3D passage portal"};
  }

  std::set<PassageTraversalId> traversal_ids;
  for (const PassageTraversalEdge& edge : traversal_edges_) {
    const bool portal_geometry_matches =
        portal_centers.contains(edge.entry_portal_id) &&
        portal_centers.contains(edge.exit_portal_id) && !edge.centerline.empty() &&
        distance3D(edge.centerline.front().position,
                   portal_centers.at(edge.entry_portal_id)) <= 1.0e-5 &&
        distance3D(edge.centerline.back().position,
                   portal_centers.at(edge.exit_portal_id)) <= 1.0e-5;
    const bool centerline_valid =
        edge.centerline.size() >= 2U &&
        edge.centerline.size() <= kMaximumGeometryPointCount &&
        std::ranges::all_of(edge.centerline, [](const RouteSample3D& sample) {
          return finite(sample.position);
        });
    const bool valid =
        !edge.id.empty() && traversal_ids.insert(edge.id).second &&
        region_ids.contains(edge.region_id) &&
        portal_regions.contains(edge.entry_portal_id) &&
        portal_regions.contains(edge.exit_portal_id) &&
        portal_regions.at(edge.entry_portal_id) == edge.region_id &&
        portal_regions.at(edge.exit_portal_id) == edge.region_id &&
        edge.entry_portal_id != edge.exit_portal_id && portal_geometry_matches &&
        centerline_valid && std::isfinite(edge.min_z_m) &&
        std::isfinite(edge.max_z_m) && edge.max_z_m > edge.min_z_m &&
        std::isfinite(edge.width_m) && edge.width_m > 0.0 &&
        std::isfinite(edge.height_m) && edge.height_m > 0.0 &&
        std::isfinite(edge.minimum_clearance_m) && edge.minimum_clearance_m > 0.0 &&
        std::isfinite(edge.speed_limit_mps) && edge.speed_limit_mps > 0.0;
    if (!valid) {
      throw std::invalid_argument{"invalid FreeSpaceTopology3D passage traversal"};
    }
  }
}

} // namespace drone_city_nav
