#pragma once

#include "drone_city_nav/passage_ids.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace drone_city_nav {

struct PassagePortal {
  PassagePortalId id;
  FreeSpaceRegionId region_id;
  Point3 center{};
  Vec3 outward_normal{};
  std::vector<Point3> opening_polygon;
  std::vector<GridIndex3D> surface_voxels;
  std::vector<Point3> traversable_anchors;
  Vec3 local_u_axis{};
  Vec3 local_v_axis{};
  double minimum_clearance_m{0.0};
  double mean_clearance_m{0.0};
  double maximum_clearance_m{0.0};
};

struct FreeSpaceRegion {
  FreeSpaceRegionId id;
  Point3 representative{};
  double maximum_clearance_m{0.0};
  std::vector<PassagePortalId> portal_ids;
};

struct PassageSegment {
  PassageSegmentId id;
  std::vector<RouteSample3D> centerline;
  std::vector<PassagePortalId> endpoint_portal_ids;
  std::vector<PassageSegmentId> first_endpoint_neighbors;
  std::vector<PassageSegmentId> second_endpoint_neighbors;
  double minimum_clearance_m{0.0};
  double speed_limit_mps{0.0};
};

struct PassageTraversalEdge {
  PassageTraversalId id;
  FreeSpaceRegionId region_id;
  PassagePortalId entry_portal_id;
  PassagePortalId exit_portal_id;
  std::vector<RouteSample3D> centerline;
  Point3 entry{};
  Point3 exit{};
  double min_z_m{0.0};
  double max_z_m{0.0};
  double width_m{0.0};
  double height_m{0.0};
  double minimum_clearance_m{0.0};
  double speed_limit_mps{0.0};
};

} // namespace drone_city_nav
