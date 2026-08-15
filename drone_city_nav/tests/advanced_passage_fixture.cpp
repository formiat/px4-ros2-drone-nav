#include "advanced_passage_fixture.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_city_nav::test {
namespace {

constexpr GridBounds3D kBounds{0.0, 0.0, 0.0, 0.5, 64, 64, 64};
constexpr double kRoomRadiusM{5.0};
constexpr double kTunnelRadiusM{2.5};

class DenseCarvingVolume {
public:
  DenseCarvingVolume()
      : solid_(static_cast<std::size_t>(kBounds.width_cells) *
                   static_cast<std::size_t>(kBounds.height_cells) *
                   static_cast<std::size_t>(kBounds.depth_cells),
               true) {
  }

  void carve(const std::function<bool(const Point3&)>& predicate) {
    for (int z = 0; z < kBounds.depth_cells; ++z) {
      for (int y = 0; y < kBounds.height_cells; ++y) {
        for (int x = 0; x < kBounds.width_cells; ++x) {
          const GridIndex3D index{x, y, z};
          if (predicate(cellCenter(index))) {
            solid_[linearIndex(index)] = false;
          }
        }
      }
    }
  }

  void carveSphere(const Point3& center, const double radius_m) {
    const double radius_squared_m2 = radius_m * radius_m;
    carve([center, radius_squared_m2](const Point3& point) {
      const double dx = point.x - center.x;
      const double dy = point.y - center.y;
      const double dz = point.z - center.z;
      return dx * dx + dy * dy + dz * dz <= radius_squared_m2;
    });
  }

  void carvePolyline(const std::vector<Point3>& points, const double radius_m) {
    if (points.size() < 2U) {
      throw std::invalid_argument{"fixture polyline requires at least two points"};
    }
    const double radius_squared_m2 = radius_m * radius_m;
    carve([points, radius_squared_m2](const Point3& point) {
      double minimum_distance_squared_m2 = std::numeric_limits<double>::infinity();
      for (std::size_t index = 1U; index < points.size(); ++index) {
        minimum_distance_squared_m2 = std::min(
            minimum_distance_squared_m2,
            pointSegmentDistanceSquared(point, points[index - 1U], points[index]));
      }
      return minimum_distance_squared_m2 <= radius_squared_m2;
    });
  }

  void carveBox(const Point3& minimum, const Point3& maximum) {
    carve([minimum, maximum](const Point3& point) {
      return point.x >= minimum.x && point.x <= maximum.x && point.y >= minimum.y &&
             point.y <= maximum.y && point.z >= minimum.z && point.z <= maximum.z;
    });
  }

  [[nodiscard]] OccupancyGrid3D build(const std::uint64_t fingerprint) const {
    OccupancyGrid3D occupancy{kBounds, fingerprint};
    for (int z = 0; z < kBounds.depth_cells; ++z) {
      for (int y = 0; y < kBounds.height_cells; ++y) {
        for (int x = 0; x < kBounds.width_cells; ++x) {
          const GridIndex3D index{x, y, z};
          if (solid_[linearIndex(index)]) {
            occupancy.setOccupied(index);
          }
        }
      }
    }
    return occupancy;
  }

private:
  [[nodiscard]] static Point3 cellCenter(const GridIndex3D index) noexcept {
    return Point3{(static_cast<double>(index.x) + 0.5) * kBounds.resolution_m,
                  (static_cast<double>(index.y) + 0.5) * kBounds.resolution_m,
                  (static_cast<double>(index.z) + 0.5) * kBounds.resolution_m};
  }

  [[nodiscard]] static std::size_t linearIndex(const GridIndex3D index) noexcept {
    return (static_cast<std::size_t>(index.z) *
                static_cast<std::size_t>(kBounds.height_cells) +
            static_cast<std::size_t>(index.y)) *
               static_cast<std::size_t>(kBounds.width_cells) +
           static_cast<std::size_t>(index.x);
  }

  [[nodiscard]] static double
  pointSegmentDistanceSquared(const Point3& point, const Point3& first,
                              const Point3& second) noexcept {
    const Vec3 segment{second.x - first.x, second.y - first.y, second.z - first.z};
    const Vec3 offset{point.x - first.x, point.y - first.y, point.z - first.z};
    const double length_squared_m2 =
        segment.x * segment.x + segment.y * segment.y + segment.z * segment.z;
    const double projection = std::clamp(
        (offset.x * segment.x + offset.y * segment.y + offset.z * segment.z) /
            length_squared_m2,
        0.0, 1.0);
    const double dx = offset.x - projection * segment.x;
    const double dy = offset.y - projection * segment.y;
    const double dz = offset.z - projection * segment.z;
    return dx * dx + dy * dy + dz * dz;
  }

  std::vector<bool> solid_;
};

[[nodiscard]] std::uint64_t
fixtureFingerprint(const AdvancedPassageFixtureKind kind) noexcept {
  return UINT64_C(0xAD7A000000000000) + static_cast<std::uint64_t>(kind);
}

[[nodiscard]] AdvancedPassageFixture slopedTunnelFixture() {
  const std::vector<Point3> path{
      {7.0, 16.0, 7.0}, {13.0, 16.0, 11.5}, {19.0, 16.0, 17.5}, {25.0, 16.0, 23.0}};
  DenseCarvingVolume volume;
  volume.carveSphere(path.front(), kRoomRadiusM);
  volume.carveSphere(path.back(), kRoomRadiusM);
  volume.carvePolyline(path, kTunnelRadiusM);
  return AdvancedPassageFixture{
      .kind = AdvancedPassageFixtureKind::kSlopedTunnel,
      .name = "sloped_tunnel",
      .occupancy =
          volume.build(fixtureFingerprint(AdvancedPassageFixtureKind::kSlopedTunnel)),
      .expectation =
          AdvancedPassageExpectation{
              .open_space_seeds = {path.front(), path.back()},
              .raw_safe_reference_paths = {path},
          },
  };
}

[[nodiscard]] AdvancedPassageFixture verticalShaftFixture() {
  const std::vector<Point3> path{
      {16.0, 16.0, 6.0}, {16.0, 16.0, 12.0}, {16.0, 16.0, 20.0}, {16.0, 16.0, 26.0}};
  DenseCarvingVolume volume;
  volume.carveSphere(path.front(), kRoomRadiusM);
  volume.carveSphere(path.back(), kRoomRadiusM);
  volume.carvePolyline(path, kTunnelRadiusM);
  return AdvancedPassageFixture{
      .kind = AdvancedPassageFixtureKind::kVerticalShaft,
      .name = "vertical_shaft",
      .occupancy =
          volume.build(fixtureFingerprint(AdvancedPassageFixtureKind::kVerticalShaft)),
      .expectation =
          AdvancedPassageExpectation{
              .open_space_seeds = {path.front(), path.back()},
              .raw_safe_reference_paths = {path},
          },
  };
}

[[nodiscard]] AdvancedPassageFixture archTunnelFixture() {
  const Point3 first{6.0, 16.0, 8.0};
  const Point3 second{26.0, 16.0, 8.0};
  const std::vector<Point3> path{first, {12.0, 16.0, 8.0}, {20.0, 16.0, 8.0}, second};
  DenseCarvingVolume volume;
  volume.carveSphere(first, kRoomRadiusM);
  volume.carveSphere(second, kRoomRadiusM);
  volume.carve([](const Point3& point) {
    if (point.x < 9.0 || point.x > 23.0) {
      return false;
    }
    const double lateral_m = std::abs(point.y - 16.0);
    if (lateral_m > 3.0 || point.z < 4.0) {
      return false;
    }
    if (point.z <= 8.0) {
      return true;
    }
    const double normalized_y = lateral_m / 3.0;
    const double normalized_z = (point.z - 8.0) / 3.0;
    return normalized_y * normalized_y + normalized_z * normalized_z <= 1.0;
  });
  return AdvancedPassageFixture{
      .kind = AdvancedPassageFixtureKind::kArchTunnel,
      .name = "arch_tunnel",
      .occupancy =
          volume.build(fixtureFingerprint(AdvancedPassageFixtureKind::kArchTunnel)),
      .expectation =
          AdvancedPassageExpectation{
              .open_space_seeds = {first, second},
              .raw_safe_reference_paths = {path},
          },
  };
}

[[nodiscard]] AdvancedPassageFixture curvedTunnelFixture() {
  const std::vector<Point3> path{{6.0, 7.0, 8.0},
                                 {15.0, 7.0, 9.0},
                                 {22.0, 13.0, 12.0},
                                 {24.0, 23.0, 16.0},
                                 {24.0, 27.0, 17.0}};
  DenseCarvingVolume volume;
  volume.carveSphere(path.front(), kRoomRadiusM);
  volume.carveSphere(path.back(), kRoomRadiusM);
  volume.carvePolyline(path, kTunnelRadiusM);
  return AdvancedPassageFixture{
      .kind = AdvancedPassageFixtureKind::kCurvedTunnel,
      .name = "curved_tunnel",
      .occupancy =
          volume.build(fixtureFingerprint(AdvancedPassageFixtureKind::kCurvedTunnel)),
      .expectation =
          AdvancedPassageExpectation{
              .open_space_seeds = {path.front(), path.back()},
              .raw_safe_reference_paths = {path},
          },
  };
}

[[nodiscard]] AdvancedPassageFixture junctionFixture(const bool x_junction) {
  const Point3 center{16.0, 16.0, 12.0};
  std::vector<Point3> endpoints{
      {5.0, 16.0, 12.0}, {27.0, 16.0, 12.0}, {16.0, 27.0, 15.0}};
  if (x_junction) {
    endpoints.push_back({16.0, 5.0, 9.0});
  }
  DenseCarvingVolume volume;
  std::vector<std::vector<Point3>> paths;
  for (const Point3& endpoint : endpoints) {
    volume.carveSphere(endpoint, 4.25);
    paths.push_back({endpoint, center});
    volume.carvePolyline(paths.back(), kTunnelRadiusM);
  }
  const AdvancedPassageFixtureKind kind = x_junction
                                              ? AdvancedPassageFixtureKind::kXJunction
                                              : AdvancedPassageFixtureKind::kTJunction;
  return AdvancedPassageFixture{
      .kind = kind,
      .name = x_junction ? "x_junction" : "t_junction",
      .occupancy = volume.build(fixtureFingerprint(kind)),
      .expectation =
          AdvancedPassageExpectation{
              .minimum_portal_count = endpoints.size(),
              .minimum_segment_count = endpoints.size(),
              .open_space_seeds = std::move(endpoints),
              .raw_safe_reference_paths = std::move(paths),
          },
  };
}

[[nodiscard]] AdvancedPassageFixture wideHangarFixture() {
  DenseCarvingVolume volume;
  volume.carveBox({3.0, 4.0, 2.0}, {29.0, 28.0, 20.0});
  return AdvancedPassageFixture{
      .kind = AdvancedPassageFixtureKind::kWideHangar,
      .name = "wide_hangar_negative",
      .occupancy =
          volume.build(fixtureFingerprint(AdvancedPassageFixtureKind::kWideHangar)),
      .expectation =
          AdvancedPassageExpectation{
              .should_extract_passage = false,
              .minimum_portal_count = 0U,
              .minimum_segment_count = 0U,
              .open_space_seeds = {{8.0, 16.0, 10.0}, {24.0, 16.0, 10.0}},
              .raw_safe_reference_paths = {{{8.0, 16.0, 10.0},
                                            {16.0, 16.0, 10.0},
                                            {24.0, 16.0, 10.0}}},
          },
  };
}

} // namespace

std::vector<AdvancedPassageFixtureKind> advancedPassageFixtureKinds() {
  return {
      AdvancedPassageFixtureKind::kSlopedTunnel,
      AdvancedPassageFixtureKind::kVerticalShaft,
      AdvancedPassageFixtureKind::kArchTunnel,
      AdvancedPassageFixtureKind::kCurvedTunnel,
      AdvancedPassageFixtureKind::kTJunction,
      AdvancedPassageFixtureKind::kXJunction,
      AdvancedPassageFixtureKind::kWideHangar,
  };
}

AdvancedPassageFixture
buildAdvancedPassageFixture(const AdvancedPassageFixtureKind kind) {
  switch (kind) {
    case AdvancedPassageFixtureKind::kSlopedTunnel:
      return slopedTunnelFixture();
    case AdvancedPassageFixtureKind::kVerticalShaft:
      return verticalShaftFixture();
    case AdvancedPassageFixtureKind::kArchTunnel:
      return archTunnelFixture();
    case AdvancedPassageFixtureKind::kCurvedTunnel:
      return curvedTunnelFixture();
    case AdvancedPassageFixtureKind::kTJunction:
      return junctionFixture(false);
    case AdvancedPassageFixtureKind::kXJunction:
      return junctionFixture(true);
    case AdvancedPassageFixtureKind::kWideHangar:
      return wideHangarFixture();
  }
  throw std::invalid_argument{"unsupported advanced passage fixture"};
}

} // namespace drone_city_nav::test
