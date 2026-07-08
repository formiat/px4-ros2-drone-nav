#include "trajectory_diagnostics_io_internal.hpp"

namespace drone_city_nav {

using namespace trajectory_diagnostics_io_detail;

std::string finalTrajectorySamplesCsvHeader() {
  return "sample_index,s_m,x,y,z_m,tangent_x,tangent_y,curvature_1pm,"
         "arc_radius_m,left_bound_m,right_bound_m,lateral_offset_m,"
         "vertical_slope_dz_ds,vertical_speed_limit_mps,"
         "vertical_accel_limit_mps,vertical_jerk_limit_mps,"
         "vertical_constraint_active,vertical_profile_passage_id,"
         "vertical_hard_window_active,vertical_safe_min_z_m,"
         "vertical_safe_max_z_m,vertical_gate_z_m,"
         "speed_geometric_limit_mps,speed_profiled_limit_mps,speed_reason,"
         "speed_limit_source,constraint_s_m,constraint_limit_mps,"
         "profiled_time_from_start_s,profiled_time_to_finish_s";
}

std::string finalTrajectorySamplesCsvRow(const std::size_t sample_index,
                                         const TrajectoryPointSample& sample,
                                         const TrajectorySpeedSample& speed_sample,
                                         const double time_from_start_s,
                                         const double time_to_finish_s) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << sample_index << ",";
  writeCsvNumberOrEmpty(stream, sample.s_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.point.x);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.point.y);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.z_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.tangent.x);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.tangent.y);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.curvature_1pm);
  stream << ",";
  if (std::abs(sample.curvature_1pm) > kTinyCurvature) {
    writeCsvNumberOrEmpty(stream, 1.0 / std::abs(sample.curvature_1pm));
  }
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.left_bound_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.right_bound_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.lateral_offset_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.vertical_slope_dz_ds);
  stream << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.vertical_speed_limit_mps);
  stream << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.vertical_accel_limit_mps);
  stream << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.vertical_jerk_limit_mps);
  stream << "," << (sample.vertical_constraint_active ? "true" : "false") << ","
         << sample.vertical_profile_passage_id << ",";
  stream << (sample.vertical_hard_window_active ? "true" : "false") << ",";
  writeCsvNumberOrEmpty(stream, sample.vertical_safe_min_z_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.vertical_safe_max_z_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, sample.vertical_gate_z_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.geometric_limit_mps);
  stream << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.profiled_limit_mps);
  stream << "," << speedConstraintTypeName(speed_sample.reason) << ","
         << speedConstraintTypeName(speed_sample.reason) << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.constraint_s_m);
  stream << ",";
  writeCsvNumberOrEmpty(stream, speed_sample.constraint_limit_mps);
  stream << ",";
  writeCsvNumberOrEmpty(stream, time_from_start_s);
  stream << ",";
  writeCsvNumberOrEmpty(stream, time_to_finish_s);
  return stream.str();
}

} // namespace drone_city_nav
