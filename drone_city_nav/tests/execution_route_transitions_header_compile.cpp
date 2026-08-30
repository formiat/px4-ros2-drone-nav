#include "drone_city_nav/execution_route_transitions_3d.hpp"

bool executionRouteTransitionsHeaderIsSelfContained() {
  return drone_city_nav::ExecutionRouteTransitionGuard3D{}.expected_snapshot_version ==
         0U;
}
