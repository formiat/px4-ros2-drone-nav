#include "drone_city_nav/execution_route_store_3d.hpp"

bool executionRouteStoreHeaderIsSelfContained() {
  return drone_city_nav::ExecutionRoutePublicationStatus3D::kPublished !=
         drone_city_nav::ExecutionRoutePublicationStatus3D::kInvalidCandidate;
}
