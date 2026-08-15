#include "drone_city_nav/passage_volume.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] PassageVolumeConfig config() {
  return PassageVolumeConfig{
      .cross_section_spacing_m = 1.0,
      .lateral_probe_step_m = 0.5,
      .secondary_probe_step_m = 0.5,
      .maximum_cross_section_probe_m = 12.0,
      .minimum_wall_clearance_m = 0.25,
      .flight_envelope =
          FlightEnvelopeConfig{.minimum_target_z_m = 1.0, .maximum_target_z_m = 12.0},
      .footprint =
          SweptFootprintConfig{
              .radius_m = 0.5,
              .lower_extent_m = 0.25,
              .upper_extent_m = 0.25,
              .perimeter_samples = 12U,
              .radial_rings = 2U,
              .axial_samples = 3U,
              .sweep_step_m = 0.25,
          },
  };
}

[[nodiscard]] ConstrainedRouteSpan span(const double begin_station_m,
                                        const double end_station_m) {
  return ConstrainedRouteSpan{
      .passage_traversal_id = "derived_passage",
      .route_generation = 9U,
      .direction_sign = 1,
      .begin_station_m = begin_station_m,
      .end_station_m = end_station_m,
      .envelope =
          {
              RouteEnvelopeSample{
                  .station_m = begin_station_m,
                  .lateral_free_left_m = 50.0,
                  .lateral_free_right_m = 50.0,
                  .min_z_m = 0.0,
                  .max_z_m = 20.0,
                  .minimum_clearance_m = 50.0,
                  .reference_z_m = 5.5,
                  .reference_speed_mps = 10.0,
              },
              RouteEnvelopeSample{
                  .station_m = end_station_m,
                  .lateral_free_left_m = 50.0,
                  .lateral_free_right_m = 50.0,
                  .min_z_m = 0.0,
                  .max_z_m = 20.0,
                  .minimum_clearance_m = 50.0,
                  .reference_z_m = 5.5,
                  .reference_speed_mps = 10.0,
              },
          },
      .segment_spans = {},
  };
}

[[nodiscard]] OccupancyGrid3D varyingTunnel() {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, -10.0, 0.0, 1.0, 30, 20, 15}};
  for (int x = 1; x < 29; ++x) {
    for (int z = 1; z < 11; ++z) {
      occupancy.setOccupied(GridIndex3D{x, 5, z});
      occupancy.setOccupied(GridIndex3D{x, x < 15 ? 16 : 13, z});
    }
    for (int y = 5; y <= 16; ++y) {
      occupancy.setOccupied(GridIndex3D{x, y, 9});
    }
  }
  return occupancy;
}

TEST(PassageVolume, DerivesVaryingCrossSectionsFromRawOccupancy) {
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{2.5, 0.5, 5.5}, {27.5, 0.5, 5.5}}, 1.0, 10.0);
  const ConstrainedRouteSpan constrained = span(1.0, 24.0);

  const std::vector<PassageVolume> volumes = derivePassageVolumes(
      route, std::vector<ConstrainedRouteSpan>{constrained}, varyingTunnel(), config());

  ASSERT_EQ(volumes.size(), 1U);
  const PassageVolume& volume = volumes.front();
  ASSERT_TRUE(volume.raw_validated);
  EXPECT_EQ(volume.passage_traversal_id, constrained.passage_traversal_id);
  EXPECT_GE(volume.cross_sections.size(), 20U);
  EXPECT_TRUE(std::isfinite(volume.minimum_lateral_offset_m));
  EXPECT_TRUE(std::isfinite(volume.maximum_lateral_offset_m));
  EXPECT_LT(volume.minimum_physical_width_m, 12.0);
  const PassageCrossSection* const wide = nearestPassageCrossSection(volume, 5.0);
  const PassageCrossSection* const narrow = nearestPassageCrossSection(volume, 20.0);
  ASSERT_NE(wide, nullptr);
  ASSERT_NE(narrow, nullptr);
  EXPECT_GT(wide->maximum_lateral_offset_m, narrow->maximum_lateral_offset_m + 1.0);
  EXPECT_LE(volume.maximum_lateral_offset_m, narrow->maximum_lateral_offset_m + 1.0e-9);
  EXPECT_LT(volume.maximum_secondary_offset_m, 4.0);
}

TEST(PassageVolume, BuildsOrthonormalFramesForSlopedRoutes) {
  const OccupancyGrid3D occupancy{GridBounds3D{-5.0, -10.0, 0.0, 1.0, 40, 20, 20}};
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{2.0, 0.0, 3.0}, {20.0, 0.0, 8.0}}, 1.0, 10.0);
  const ConstrainedRouteSpan constrained = span(1.0, route.back().station_m - 1.0);

  const std::vector<PassageVolume> volumes = derivePassageVolumes(
      route, std::vector<ConstrainedRouteSpan>{constrained}, occupancy, config());

  ASSERT_EQ(volumes.size(), 1U);
  ASSERT_TRUE(volumes.front().raw_validated);
  for (const PassageCrossSection& section : volumes.front().cross_sections) {
    const auto dot = [](const Vec3& first, const Vec3& second) {
      return first.x * second.x + first.y * second.y + first.z * second.z;
    };
    EXPECT_NEAR(dot(section.tangent, section.lateral_axis), 0.0, 1.0e-9);
    EXPECT_NEAR(dot(section.tangent, section.secondary_axis), 0.0, 1.0e-9);
    EXPECT_NEAR(dot(section.lateral_axis, section.secondary_axis), 0.0, 1.0e-9);
  }
}

TEST(PassageVolume, KeepsPhysicalCrossSectionWhenCooperativeMarginDoesNotFit) {
  PassageVolumeConfig narrow_coordination = config();
  narrow_coordination.maximum_cross_section_probe_m = 2.0;
  narrow_coordination.minimum_wall_clearance_m = 3.0;
  const OccupancyGrid3D occupancy{GridBounds3D{-5.0, -5.0, 0.0, 1.0, 30, 10, 12}};
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{2.5, 0.5, 5.5}, {17.5, 0.5, 5.5}}, 1.0, 10.0);

  const std::vector<PassageVolume> volumes =
      derivePassageVolumes(route, std::vector<ConstrainedRouteSpan>{span(1.0, 14.0)},
                           occupancy, narrow_coordination);

  ASSERT_EQ(volumes.size(), 1U);
  const PassageVolume& volume = volumes.front();
  ASSERT_TRUE(volume.raw_validated);
  EXPECT_DOUBLE_EQ(volume.minimum_lateral_offset_m, 0.0);
  EXPECT_DOUBLE_EQ(volume.maximum_lateral_offset_m, 0.0);
  EXPECT_DOUBLE_EQ(volume.minimum_secondary_offset_m, 0.0);
  EXPECT_DOUBLE_EQ(volume.maximum_secondary_offset_m, 0.0);
  ASSERT_FALSE(volume.cross_sections.empty());
  for (const PassageCrossSection& section : volume.cross_sections) {
    EXPECT_LT(section.minimum_lateral_offset_m, 0.0);
    EXPECT_GT(section.maximum_lateral_offset_m, 0.0);
    EXPECT_LT(section.minimum_secondary_offset_m, 0.0);
    EXPECT_GT(section.maximum_secondary_offset_m, 0.0);
  }
}

TEST(PassageVolume, RejectsCrossSectionWhoseRouteCenterIsOccupied) {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, -5.0, 0.0, 1.0, 20, 10, 12}};
  occupancy.setOccupied(GridIndex3D{10, 5, 5});
  const std::vector<RouteSample3D> route =
      sampleRoute3D(std::vector<Point3>{{2.5, 0.5, 5.5}, {17.5, 0.5, 5.5}}, 1.0, 10.0);
  const ConstrainedRouteSpan constrained = span(1.0, 14.0);

  const std::vector<PassageVolume> volumes = derivePassageVolumes(
      route, std::vector<ConstrainedRouteSpan>{constrained}, occupancy, config());

  ASSERT_EQ(volumes.size(), 1U);
  EXPECT_FALSE(volumes.front().raw_validated);
  EXPECT_TRUE(std::ranges::any_of(
      volumes.front().cross_sections,
      [](const PassageCrossSection& section) { return !section.raw_validated; }));
}

TEST(PassageVolume, RejectsNonFiniteConfiguration) {
  PassageVolumeConfig invalid = config();
  invalid.minimum_wall_clearance_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(passageVolumeConfigIsValid(invalid));
}

} // namespace
} // namespace drone_city_nav
