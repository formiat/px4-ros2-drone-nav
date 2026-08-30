#include "drone_city_nav/execution_route_model_3d.hpp"

bool executionRouteModelHeaderIsSelfContained() {
  return drone_city_nav::RouteInstanceId3D{1U}.valid();
}
