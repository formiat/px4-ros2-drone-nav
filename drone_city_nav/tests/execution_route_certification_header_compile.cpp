#include "drone_city_nav/execution_route_certification_3d.hpp"

bool executionRouteCertificationHeaderIsSelfContained() {
  return drone_city_nav::FiniteExecutionCertification3D{}.trajectory_revision == 0U;
}
