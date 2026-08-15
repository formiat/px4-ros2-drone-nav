#include "drone_city_nav/collision_voxelizer.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Plane.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

struct Options {
  std::filesystem::path sdf;
  std::filesystem::path output;
  std::optional<std::filesystem::path> report;
  double resolution_m{0.5};
  double margin_m{2.0};
  std::size_t maximum_voxel_count{500000000U};
};

enum class SourceKind : std::uint8_t {
  kMesh,
  kBox,
  kPlane
};

struct CollisionSource {
  SourceKind kind{SourceKind::kMesh};
  gz::math::Pose3d pose;
  std::filesystem::path mesh_path;
  std::string submesh;
  gz::math::Vector3d scale{1.0, 1.0, 1.0};
  gz::math::Vector3d box_size;
  gz::math::Vector2d plane_size;
  gz::math::Vector3d plane_normal;
};

struct Bounds {
  gz::math::Vector3d minimum{std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity()};
  gz::math::Vector3d maximum{-std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity(),
                             -std::numeric_limits<double>::infinity()};

  void include(const gz::math::Vector3d& point) {
    minimum.X(std::min(minimum.X(), point.X()));
    minimum.Y(std::min(minimum.Y(), point.Y()));
    minimum.Z(std::min(minimum.Z(), point.Z()));
    maximum.X(std::max(maximum.X(), point.X()));
    maximum.Y(std::max(maximum.Y(), point.Y()));
    maximum.Z(std::max(maximum.Z(), point.Z()));
  }
};

struct VoxelizationSummary {
  std::size_t collision_sources{0U};
  std::size_t mesh_sources{0U};
  std::size_t box_sources{0U};
  std::size_t plane_sources{0U};
  std::size_t triangles{0U};
  std::size_t tested_voxels{0U};
};

[[nodiscard]] Options parseOptions(const std::vector<std::string_view>& arguments) {
  Options options;
  for (std::size_t index = 0U; index < arguments.size(); ++index) {
    const std::string_view argument = arguments.at(index);
    const auto value = [&]() -> std::string_view {
      if (index + 1U >= arguments.size()) {
        throw std::invalid_argument{"missing value after " + std::string{argument}};
      }
      return arguments.at(++index);
    };
    if (argument == "--sdf") {
      options.sdf = value();
    } else if (argument == "--output") {
      options.output = value();
    } else if (argument == "--report") {
      options.report = std::filesystem::path{value()};
    } else if (argument == "--resolution-m") {
      options.resolution_m = std::stod(std::string{value()});
    } else if (argument == "--margin-m") {
      options.margin_m = std::stod(std::string{value()});
    } else if (argument == "--maximum-voxel-count") {
      options.maximum_voxel_count =
          static_cast<std::size_t>(std::stoull(std::string{value()}));
    } else {
      throw std::invalid_argument{"unknown argument: " + std::string{argument}};
    }
  }
  if (options.sdf.empty() || options.output.empty() || !(options.resolution_m > 0.0) ||
      options.margin_m < 0.0 || options.maximum_voxel_count == 0U) {
    throw std::invalid_argument{
        "usage: voxelize_sdf_collisions --sdf PATH --output PATH "
        "[--report PATH] [--resolution-m 0.5] [--margin-m 2] "
        "[--maximum-voxel-count 500000000]"};
  }
  return options;
}

[[nodiscard]] gz::math::Vector3d transformPoint(const gz::math::Pose3d& pose,
                                                const gz::math::Vector3d& point) {
  return pose.Pos() + pose.Rot().RotateVector(point);
}

[[nodiscard]] const gz::common::Mesh& loadMesh(const std::filesystem::path& path) {
  const gz::common::Mesh* mesh =
      gz::common::MeshManager::Instance()->Load(path.string());
  if (mesh == nullptr) {
    throw std::runtime_error{"failed to load collision mesh: " + path.string()};
  }
  return *mesh;
}

[[nodiscard]] std::vector<const gz::common::SubMesh*>
selectedSubmeshes(const gz::common::Mesh& mesh, const std::string& selected) {
  std::vector<const gz::common::SubMesh*> result;
  if (!selected.empty()) {
    const std::shared_ptr<gz::common::SubMesh> submesh =
        mesh.SubMeshByName(selected).lock();
    if (submesh == nullptr) {
      throw std::runtime_error{"collision submesh does not exist: " + selected};
    }
    result.push_back(submesh.get());
    return result;
  }
  result.reserve(mesh.SubMeshCount());
  for (unsigned int index = 0U; index < mesh.SubMeshCount(); ++index) {
    const std::shared_ptr<gz::common::SubMesh> submesh =
        mesh.SubMeshByIndex(index).lock();
    if (submesh == nullptr) {
      throw std::runtime_error{"collision mesh contains an expired submesh"};
    }
    result.push_back(submesh.get());
  }
  return result;
}

[[nodiscard]] std::array<gz::math::Vector3d, 8U>
boxCorners(const gz::math::Vector3d& minimum, const gz::math::Vector3d& maximum) {
  return {gz::math::Vector3d{minimum.X(), minimum.Y(), minimum.Z()},
          gz::math::Vector3d{maximum.X(), minimum.Y(), minimum.Z()},
          gz::math::Vector3d{minimum.X(), maximum.Y(), minimum.Z()},
          gz::math::Vector3d{maximum.X(), maximum.Y(), minimum.Z()},
          gz::math::Vector3d{minimum.X(), minimum.Y(), maximum.Z()},
          gz::math::Vector3d{maximum.X(), minimum.Y(), maximum.Z()},
          gz::math::Vector3d{minimum.X(), maximum.Y(), maximum.Z()},
          gz::math::Vector3d{maximum.X(), maximum.Y(), maximum.Z()}};
}

[[nodiscard]] std::array<gz::math::Vector3d, 4U>
planeCorners(const CollisionSource& source) {
  const gz::math::Vector3d normal = source.plane_normal.Normalized();
  const gz::math::Vector3d reference = std::abs(normal.Z()) < 0.9
                                           ? gz::math::Vector3d::UnitZ
                                           : gz::math::Vector3d::UnitX;
  const gz::math::Vector3d tangent = normal.Cross(reference).Normalized();
  const gz::math::Vector3d bitangent = normal.Cross(tangent).Normalized();
  const gz::math::Vector3d horizontal = 0.5 * source.plane_size.X() * tangent;
  const gz::math::Vector3d vertical = 0.5 * source.plane_size.Y() * bitangent;
  return {-horizontal - vertical, horizontal - vertical, -horizontal + vertical,
          horizontal + vertical};
}

void includeSourceBounds(const CollisionSource& source, Bounds& bounds) {
  gz::math::Vector3d minimum;
  gz::math::Vector3d maximum;
  if (source.kind == SourceKind::kMesh) {
    const gz::common::Mesh& mesh = loadMesh(source.mesh_path);
    if (source.submesh.empty()) {
      minimum = mesh.Min();
      maximum = mesh.Max();
    } else {
      const auto submeshes = selectedSubmeshes(mesh, source.submesh);
      minimum = submeshes.front()->Min();
      maximum = submeshes.front()->Max();
    }
    minimum *= source.scale;
    maximum *= source.scale;
  } else if (source.kind == SourceKind::kBox) {
    minimum = -0.5 * source.box_size;
    maximum = 0.5 * source.box_size;
  } else {
    for (const gz::math::Vector3d& corner : planeCorners(source)) {
      bounds.include(transformPoint(source.pose, corner));
    }
    return;
  }
  for (const gz::math::Vector3d& corner : boxCorners(minimum, maximum)) {
    bounds.include(transformPoint(source.pose, corner));
  }
}

[[nodiscard]] std::vector<CollisionSource>
loadSources(const std::filesystem::path& path) {
  sdf::Root root;
  const sdf::Errors errors = root.Load(path.string());
  if (!errors.empty()) {
    std::string message{"failed to parse materialized SDF"};
    for (const sdf::Error& error : errors) {
      message += "\n" + error.Message();
    }
    throw std::runtime_error{message};
  }
  if (root.WorldCount() != 1U) {
    throw std::runtime_error{"materialized SDF must contain exactly one world"};
  }
  const sdf::World* world = root.WorldByIndex(0U);
  if (world == nullptr) {
    throw std::runtime_error{"materialized SDF world is unavailable"};
  }

  std::vector<CollisionSource> sources;
  for (std::uint64_t model_index = 0U; model_index < world->ModelCount();
       ++model_index) {
    const sdf::Model* model = world->ModelByIndex(model_index);
    if (model == nullptr || !model->Static()) {
      throw std::runtime_error{"materialized world contains a non-static model"};
    }
    for (std::uint64_t link_index = 0U; link_index < model->LinkCount(); ++link_index) {
      const sdf::Link* link = model->LinkByIndex(link_index);
      if (link == nullptr) {
        throw std::runtime_error{"materialized model contains an invalid link"};
      }
      for (std::uint64_t collision_index = 0U; collision_index < link->CollisionCount();
           ++collision_index) {
        const sdf::Collision* collision = link->CollisionByIndex(collision_index);
        if (collision == nullptr || collision->Geom() == nullptr) {
          throw std::runtime_error{"materialized link contains an invalid collision"};
        }
        CollisionSource source;
        source.pose = model->RawPose() * link->RawPose() * collision->RawPose();
        const sdf::Geometry& geometry = *collision->Geom();
        if (geometry.Type() == sdf::GeometryType::MESH) {
          const sdf::Mesh* mesh = geometry.MeshShape();
          if (mesh == nullptr || mesh->CenterSubmesh()) {
            throw std::runtime_error{"unsupported centered or invalid collision mesh"};
          }
          source.kind = SourceKind::kMesh;
          source.mesh_path = mesh->Uri();
          source.submesh = mesh->Submesh();
          source.scale = mesh->Scale();
          if (!source.mesh_path.is_absolute() || source.scale.X() <= 0.0 ||
              source.scale.Y() <= 0.0 || source.scale.Z() <= 0.0) {
            throw std::runtime_error{"collision mesh must use an absolute path and "
                                     "positive scale"};
          }
        } else if (geometry.Type() == sdf::GeometryType::BOX) {
          const sdf::Box* box = geometry.BoxShape();
          if (box == nullptr) {
            throw std::runtime_error{"invalid collision box"};
          }
          source.kind = SourceKind::kBox;
          source.box_size = box->Size();
        } else if (geometry.Type() == sdf::GeometryType::PLANE) {
          const sdf::Plane* plane = geometry.PlaneShape();
          if (plane == nullptr || plane->Size().X() <= 0.0 ||
              plane->Size().Y() <= 0.0 || plane->Normal().Length() <= 1.0e-9) {
            throw std::runtime_error{"invalid collision plane"};
          }
          source.kind = SourceKind::kPlane;
          source.plane_size = plane->Size();
          source.plane_normal = plane->Normal();
        } else {
          throw std::runtime_error{
              "unsupported collision geometry in static-map import"};
        }
        sources.push_back(std::move(source));
      }
    }
  }
  if (sources.empty()) {
    throw std::runtime_error{"materialized SDF contains no collision geometry"};
  }
  return sources;
}

[[nodiscard]] drone_city_nav::Point3 point(const gz::math::Vector3d& value) {
  return {value.X(), value.Y(), value.Z()};
}

[[nodiscard]] std::vector<drone_city_nav::CollisionTriangle3D>
meshTriangles(const CollisionSource& source) {
  const gz::common::Mesh& mesh = loadMesh(source.mesh_path);
  std::vector<drone_city_nav::CollisionTriangle3D> triangles;
  for (const gz::common::SubMesh* submesh : selectedSubmeshes(mesh, source.submesh)) {
    if (submesh->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES ||
        submesh->IndexCount() % 3U != 0U) {
      throw std::runtime_error{"collision mesh must contain indexed triangles"};
    }
    triangles.reserve(triangles.size() + submesh->IndexCount() / 3U);
    for (unsigned int index = 0U; index < submesh->IndexCount(); index += 3U) {
      std::array<gz::math::Vector3d, 3U> vertices;
      for (unsigned int vertex = 0U; vertex < 3U; ++vertex) {
        const int mesh_index = submesh->Index(index + vertex);
        if (mesh_index < 0) {
          throw std::runtime_error{"collision mesh contains a negative index"};
        }
        gz::math::Vector3d value =
            submesh->Vertex(static_cast<unsigned int>(mesh_index));
        value *= source.scale;
        vertices.at(vertex) = transformPoint(source.pose, value);
      }
      triangles.push_back({point(vertices[0]), point(vertices[1]), point(vertices[2])});
    }
  }
  return triangles;
}

[[nodiscard]] std::vector<drone_city_nav::CollisionTriangle3D>
boxTriangles(const CollisionSource& source) {
  const auto corners = boxCorners(-0.5 * source.box_size, 0.5 * source.box_size);
  constexpr std::array<std::array<std::size_t, 3U>, 12U> faces{{
      {0U, 2U, 1U},
      {1U, 2U, 3U},
      {4U, 5U, 6U},
      {5U, 7U, 6U},
      {0U, 1U, 4U},
      {1U, 5U, 4U},
      {2U, 6U, 3U},
      {3U, 6U, 7U},
      {0U, 4U, 2U},
      {2U, 4U, 6U},
      {1U, 3U, 5U},
      {3U, 7U, 5U},
  }};
  std::vector<drone_city_nav::CollisionTriangle3D> triangles;
  triangles.reserve(faces.size());
  for (const auto& face : faces) {
    triangles.push_back({point(transformPoint(source.pose, corners.at(face.at(0U)))),
                         point(transformPoint(source.pose, corners.at(face.at(1U)))),
                         point(transformPoint(source.pose, corners.at(face.at(2U))))});
  }
  return triangles;
}

[[nodiscard]] std::vector<drone_city_nav::CollisionTriangle3D>
planeTriangles(const CollisionSource& source) {
  const auto corners = planeCorners(source);
  return {{point(transformPoint(source.pose, corners[0])),
           point(transformPoint(source.pose, corners[1])),
           point(transformPoint(source.pose, corners[2]))},
          {point(transformPoint(source.pose, corners[1])),
           point(transformPoint(source.pose, corners[3])),
           point(transformPoint(source.pose, corners[2]))}};
}

[[nodiscard]] std::vector<std::filesystem::path>
meshPaths(const std::vector<CollisionSource>& sources) {
  std::vector<std::filesystem::path> mesh_paths;
  mesh_paths.reserve(sources.size());
  for (const CollisionSource& source : sources) {
    if (source.kind == SourceKind::kMesh) {
      mesh_paths.push_back(source.mesh_path);
    }
  }
  return mesh_paths;
}

[[nodiscard]] drone_city_nav::GridBounds3D gridBounds(const Bounds& bounds,
                                                      const Options& options) {
  if (!std::isfinite(bounds.minimum.X()) || !std::isfinite(bounds.minimum.Y()) ||
      !std::isfinite(bounds.minimum.Z()) || !std::isfinite(bounds.maximum.X()) ||
      !std::isfinite(bounds.maximum.Y()) || !std::isfinite(bounds.maximum.Z())) {
    throw std::runtime_error{"collision geometry has non-finite bounds"};
  }
  const auto origin = [&](const double minimum) {
    return std::floor((minimum - options.margin_m) / options.resolution_m) *
           options.resolution_m;
  };
  const auto dimension = [&](const double minimum, const double maximum) {
    const double cells =
        std::ceil((maximum + options.margin_m - minimum) / options.resolution_m);
    if (!(cells > 0.0) ||
        cells > static_cast<double>(std::numeric_limits<int>::max())) {
      throw std::runtime_error{"static-map dimension is outside the supported range"};
    }
    return static_cast<int>(cells);
  };
  drone_city_nav::GridBounds3D result;
  result.origin_x = origin(bounds.minimum.X());
  result.origin_y = origin(bounds.minimum.Y());
  result.origin_z = origin(bounds.minimum.Z());
  result.resolution_m = options.resolution_m;
  result.width_cells = dimension(result.origin_x, bounds.maximum.X());
  result.height_cells = dimension(result.origin_y, bounds.maximum.Y());
  result.depth_cells = dimension(result.origin_z, bounds.maximum.Z());
  const auto checked_product = [](const std::size_t lhs, const std::size_t rhs) {
    if (rhs != 0U && lhs > std::numeric_limits<std::size_t>::max() / rhs) {
      throw std::runtime_error{"static-map voxel count overflows size_t"};
    }
    return lhs * rhs;
  };
  const std::size_t voxel_count =
      checked_product(checked_product(static_cast<std::size_t>(result.width_cells),
                                      static_cast<std::size_t>(result.height_cells)),
                      static_cast<std::size_t>(result.depth_cells));
  if (voxel_count > options.maximum_voxel_count) {
    throw std::runtime_error{"static-map grid " + std::to_string(result.width_cells) +
                             "x" + std::to_string(result.height_cells) + "x" +
                             std::to_string(result.depth_cells) + " contains " +
                             std::to_string(voxel_count) +
                             " voxels and exceeds --maximum-voxel-count"};
  }
  return result;
}

void writeReport(const std::filesystem::path& path,
                 const drone_city_nav::OccupancyGrid3D& occupancy,
                 const VoxelizationSummary& summary) {
  std::ofstream stream{path};
  if (!stream) {
    throw std::runtime_error{"failed to create voxelization report: " + path.string()};
  }
  const auto& bounds = occupancy.bounds();
  stream << std::setprecision(12)
         << "{\n  \"schema\": \"drone_city_nav_collision_voxelization_v1\",\n"
         << "  \"resolution_m\": " << bounds.resolution_m << ",\n"
         << "  \"origin_m\": [" << bounds.origin_x << ", " << bounds.origin_y << ", "
         << bounds.origin_z << "],\n"
         << "  \"dimensions\": [" << bounds.width_cells << ", " << bounds.height_cells
         << ", " << bounds.depth_cells << "],\n"
         << "  \"collision_sources\": " << summary.collision_sources << ",\n"
         << "  \"mesh_sources\": " << summary.mesh_sources << ",\n"
         << "  \"box_sources\": " << summary.box_sources << ",\n"
         << "  \"plane_sources\": " << summary.plane_sources << ",\n"
         << "  \"triangles\": " << summary.triangles << ",\n"
         << "  \"tested_voxels\": " << summary.tested_voxels << ",\n"
         << "  \"occupied_voxels\": " << occupancy.occupiedVoxelCount() << ",\n"
         << "  \"occupied_chunks\": " << occupancy.occupiedChunkCount() << ",\n"
         << "  \"fingerprint\": " << occupancy.fingerprint() << "\n}\n";
}

} // namespace

int main(const int argc, char** argv) {
  try {
    gz::common::Console::SetVerbosity(0);
    std::vector<std::string_view> arguments;
    arguments.reserve(argc > 1 ? static_cast<std::size_t>(argc - 1) : 0U);
    for (int index = 1; index < argc; ++index) {
      // The C main entry point exposes arguments as a pointer array.
      // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
      arguments.emplace_back(argv[index]);
    }
    const Options options = parseOptions(arguments);
    const std::vector<CollisionSource> sources = loadSources(options.sdf);
    Bounds world_bounds;
    for (const CollisionSource& source : sources) {
      includeSourceBounds(source, world_bounds);
    }
    drone_city_nav::OccupancyGrid3D occupancy{
        gridBounds(world_bounds, options),
        drone_city_nav::fingerprintCollisionInputs(options.sdf, meshPaths(sources))};
    VoxelizationSummary summary{.collision_sources = sources.size()};
    for (const CollisionSource& source : sources) {
      std::vector<drone_city_nav::CollisionTriangle3D> triangles;
      if (source.kind == SourceKind::kMesh) {
        ++summary.mesh_sources;
        triangles = meshTriangles(source);
      } else if (source.kind == SourceKind::kBox) {
        ++summary.box_sources;
        triangles = boxTriangles(source);
      } else {
        ++summary.plane_sources;
        triangles = planeTriangles(source);
      }
      const drone_city_nav::CollisionVoxelizationStats stats =
          drone_city_nav::voxelizeCollisionTriangles(triangles, occupancy);
      summary.triangles += stats.triangles;
      summary.tested_voxels += stats.tested_voxels;
    }
    occupancy.write(options.output);
    if (options.report.has_value()) {
      writeReport(*options.report, occupancy, summary);
    }
    const auto& bounds = occupancy.bounds();
    std::cout << "SDF_COLLISIONS_VOXELIZED output=" << options.output
              << " resolution_m=" << bounds.resolution_m
              << " dimensions=" << bounds.width_cells << 'x' << bounds.height_cells
              << 'x' << bounds.depth_cells << " sources=" << summary.collision_sources
              << " triangles=" << summary.triangles
              << " occupied_voxels=" << occupancy.occupiedVoxelCount()
              << " occupied_chunks=" << occupancy.occupiedChunkCount() << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "voxelize_sdf_collisions: " << error.what() << '\n';
    return 1;
  }
}
