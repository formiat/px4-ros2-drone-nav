#include "drone_city_nav/corridor_samples_io.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>

namespace drone_city_nav {

void writeCsvNumberOrEmpty(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
  }
}

bool writeCorridorSamplesCsv(std::ostream& stream,
                             const TrajectoryPlannerResult& result,
                             const std::string_view source_label,
                             const std::uint64_t candidate_path_id) {
  if (!stream.good()) {
    return false;
  }

  stream << std::setprecision(9);
  stream << "# source=" << source_label << " candidate_path_id=" << candidate_path_id
         << " status=" << trajectoryPlannerStatusName(result.stats.status)
         << " valid=" << (result.valid ? "true" : "false") << "\n";
  stream << "sample_index,s_m,route_x,route_y,center_x,center_y,tangent_x,"
            "tangent_y,normal_x,normal_y,left_bound_m,right_bound_m,width_m,"
            "clearance_m,center_recovery_m\n";
  for (std::size_t i = 0U; i < result.corridor_samples.size(); ++i) {
    const CorridorSample& sample = result.corridor_samples[i];
    stream << i << ",";
    writeCsvNumberOrEmpty(stream, sample.s_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.route_center.x);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.route_center.y);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.center.x);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.center.y);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.tangent.x);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.tangent.y);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.normal.x);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.normal.y);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.left_bound_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.right_bound_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.left_bound_m + sample.right_bound_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.clearance_m);
    stream << ",";
    writeCsvNumberOrEmpty(stream, sample.center_recovery_m);
    stream << "\n";
  }
  return stream.good();
}

bool writeCorridorSamplesCsvFile(const std::filesystem::path& path,
                                 const TrajectoryPlannerResult& result,
                                 const std::string_view source_label,
                                 const std::uint64_t candidate_path_id) {
  std::ofstream stream{path, std::ios::out | std::ios::trunc};
  if (!stream.is_open()) {
    return false;
  }
  return writeCorridorSamplesCsv(stream, result, source_label, candidate_path_id);
}

} // namespace drone_city_nav
