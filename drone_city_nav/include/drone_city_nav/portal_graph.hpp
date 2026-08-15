#pragma once

#include "drone_city_nav/passage_ids.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <string>
#include <vector>

namespace drone_city_nav {

struct PassagePortal {
  PassagePortalId id;
  FreeSpaceRegionId region_id;
  Point3 center{};
  Vec3 outward_normal{};
  std::vector<Point3> opening_polygon;
};

struct PassageRegion {
  FreeSpaceRegionId id;
  std::vector<PassagePortalId> portal_ids;
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
