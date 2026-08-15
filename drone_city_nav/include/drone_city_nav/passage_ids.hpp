#pragma once

#include "drone_city_nav/strong_string_id.hpp"

namespace drone_city_nav {

struct FreeSpaceRegionIdTag;
struct PassagePortalIdTag;
struct PassageSegmentIdTag;
struct PassageTraversalIdTag;
struct CooperativeConflictResourceIdTag;

using FreeSpaceRegionId = StrongStringId<FreeSpaceRegionIdTag>;
using PassagePortalId = StrongStringId<PassagePortalIdTag>;
using PassageSegmentId = StrongStringId<PassageSegmentIdTag>;
using PassageTraversalId = StrongStringId<PassageTraversalIdTag>;
using CooperativeConflictResourceId = StrongStringId<CooperativeConflictResourceIdTag>;

} // namespace drone_city_nav
