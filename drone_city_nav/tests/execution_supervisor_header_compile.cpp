#include "drone_city_nav/execution_supervisor_3d.hpp"

bool executionSupervisorHeaderIsSelfContained() {
  return drone_city_nav::ExecutionLeaseCommitKind3D::kTransition !=
         drone_city_nav::ExecutionLeaseCommitKind3D::kUnchangedPlan;
}
