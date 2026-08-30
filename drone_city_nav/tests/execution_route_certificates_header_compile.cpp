#include "drone_city_nav/execution_route_certificates_3d.hpp"

bool executionRouteCertificatesHeaderIsSelfContained() {
  return drone_city_nav::FiniteExecutionKind3D::kNominal !=
         drone_city_nav::FiniteExecutionKind3D::kEmergencyBrakeTail;
}
