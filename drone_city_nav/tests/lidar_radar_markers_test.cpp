#include "drone_city_nav/lidar_radar_markers.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace drone_city_nav {

TEST(LidarRadarMarkers, BuildsExpectedRangeRingCount) {
  LidarRadarMarkerConfig config;
  config.scan_range_max_m = 35.0;

  const auto markers = buildLidarRadarMarkers(config, {});

  ASSERT_EQ(markers.markers.size(), 7U);
  EXPECT_EQ(markers.markers[0].ns, "lidar_radar_range");
  EXPECT_EQ(markers.markers[3].ns, "lidar_radar_range");
}

TEST(LidarRadarMarkers, SeparatesHitAndFreeRays) {
  LidarRadarMarkerConfig config;
  config.drone_position = Point2{1.0, 2.0};
  LidarBeamProjection free_projection;
  free_projection.status = LidarBeamProjectionStatus::kAccepted;
  free_projection.hit = false;
  free_projection.endpoint = Point2{4.0, 2.0};
  LidarBeamProjection hit_projection;
  hit_projection.status = LidarBeamProjectionStatus::kAccepted;
  hit_projection.hit = true;
  hit_projection.endpoint = Point2{1.0, 6.0};
  const std::vector<LidarBeamProjection> projections{free_projection, hit_projection};

  const auto markers = buildLidarRadarMarkers(config, projections);

  ASSERT_EQ(markers.markers.size(), 7U);
  const auto& free_rays = markers.markers[4];
  const auto& hit_rays = markers.markers[5];
  EXPECT_EQ(free_rays.ns, "lidar_radar_free_rays");
  EXPECT_EQ(hit_rays.ns, "lidar_radar_hit_rays");
  EXPECT_EQ(free_rays.points.size(), 2U);
  EXPECT_EQ(hit_rays.points.size(), 2U);
}

TEST(LidarRadarMarkers, UsesMapFrameAndConfiguredZ) {
  LidarRadarMarkerConfig config;
  config.frame_id = "map";
  config.marker_z_m = 2.5;
  config.drone_position = Point2{3.0, 4.0};
  config.heading_direction = Point2{0.0, 1.0};

  const auto markers = buildLidarRadarMarkers(config, {});

  ASSERT_FALSE(markers.markers.empty());
  for (const auto& marker : markers.markers) {
    EXPECT_EQ(marker.header.frame_id, "map");
  }
  const auto& drone = markers.markers.back();
  ASSERT_EQ(drone.points.size(), 2U);
  EXPECT_DOUBLE_EQ(drone.points.front().z, 2.5);
  EXPECT_DOUBLE_EQ(drone.points.back().y, 7.0);
}

} // namespace drone_city_nav
