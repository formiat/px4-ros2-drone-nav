#pragma once

#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <string>
#include <vector>

namespace drone_city_nav {

using PassageRegionId = std::string;
using PortalId = std::string;
using ChannelId = PassageRegionId;

struct PassagePortal {
  PortalId id;
  PassageRegionId region_id;
  Point3 center{};
  Vec3 outward_normal{};
  std::vector<Point3> opening_polygon;
};

struct PassageRegion {
  PassageRegionId id;
  std::vector<PortalId> portal_ids;
};

struct ConstrainedFreeSpaceEdge {
  ChannelId id;
  PassageRegionId region_id;
  PortalId entry_portal_id;
  PortalId exit_portal_id;
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

struct DerivedPortalGraph {
  std::vector<PassageRegion> regions;
  std::vector<PassagePortal> portals;
  std::vector<ConstrainedFreeSpaceEdge> traversal_edges;
};

} // namespace drone_city_nav
