#include "drone_city_nav/execution_plan_3d.hpp"

bool executionPlanHeaderIsSelfContained() {
  return drone_city_nav::ExecutionPlan3D{}.version == 0U;
}
