#include "drone_city_nav/execution_retention_3d.hpp"

bool executionRetentionHeaderIsSelfContained() {
  return drone_city_nav::ExecutionRetentionStatus3D::kPrepared !=
         drone_city_nav::ExecutionRetentionStatus3D::kRebuildRejected;
}
