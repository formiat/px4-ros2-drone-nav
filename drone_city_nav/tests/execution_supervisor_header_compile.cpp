#include "drone_city_nav/execution_supervisor_3d.hpp"

bool executionSupervisorHeaderIsSelfContained() {
  drone_city_nav::ExecutionSupervisor3D supervisor;
  return supervisor.authority() != nullptr;
}
