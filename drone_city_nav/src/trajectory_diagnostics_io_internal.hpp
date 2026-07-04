#pragma once

#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>

namespace drone_city_nav {
namespace trajectory_diagnostics_io_detail {

inline constexpr double kTinyCurvature = 1.0e-6;

inline void writeCsvNumberOrEmpty(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
  }
}

inline void writeJsonNumberOrNull(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
    return;
  }
  stream << "null";
}

inline void appendJsonNumber(std::ostream& stream, const std::string_view key,
                             const double value) {
  stream << ",\"" << key << "\":";
  writeJsonNumberOrNull(stream, value);
}

inline void appendJsonSize(std::ostream& stream, const std::string_view key,
                           const std::size_t value) {
  stream << ",\"" << key << "\":" << value;
}

inline void appendJsonBool(std::ostream& stream, const std::string_view key,
                           const bool value) {
  stream << ",\"" << key << "\":" << (value ? "true" : "false");
}

inline void appendJsonString(std::ostream& stream, const std::string_view key,
                             const std::string_view value) {
  stream << ",\"" << key << "\":\"" << value << "\"";
}

inline void appendJsonUint64(std::ostream& stream, const std::string_view key,
                             const std::uint64_t value) {
  stream << ",\"" << key << "\":" << value;
}

[[nodiscard]] inline std::optional<std::string_view>
jsonValueForKey(const std::string_view json, const std::string_view key) {
  const std::string pattern = "\"" + std::string{key} + "\":";
  const std::size_t key_position = json.find(pattern);
  if (key_position == std::string_view::npos) {
    return std::nullopt;
  }
  std::size_t value_begin = key_position + pattern.size();
  while (value_begin < json.size() &&
         (json[value_begin] == ' ' || json[value_begin] == '\t' ||
          json[value_begin] == '\n' || json[value_begin] == '\r')) {
    ++value_begin;
  }
  if (value_begin >= json.size()) {
    return std::nullopt;
  }

  if (json[value_begin] == '"') {
    const std::size_t string_end = json.find('"', value_begin + 1U);
    if (string_end == std::string_view::npos) {
      return std::nullopt;
    }
    return json.substr(value_begin + 1U, string_end - value_begin - 1U);
  }

  std::size_t value_end = value_begin;
  while (value_end < json.size() && json[value_end] != ',' && json[value_end] != '}') {
    ++value_end;
  }
  while (value_end > value_begin &&
         (json[value_end - 1U] == ' ' || json[value_end - 1U] == '\t' ||
          json[value_end - 1U] == '\n' || json[value_end - 1U] == '\r')) {
    --value_end;
  }
  return json.substr(value_begin, value_end - value_begin);
}

inline void parseJsonDouble(const std::string_view json, const std::string_view key,
                            double& output) {
  const std::optional<std::string_view> value = jsonValueForKey(json, key);
  if (!value.has_value()) {
    return;
  }
  if (*value == "null") {
    output = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  const std::string value_string{*value};
  char* end = nullptr;
  const double parsed = std::strtod(value_string.c_str(), &end);
  if (end != value_string.c_str() && *end == '\0') {
    output = parsed;
  }
}

inline void parseJsonSize(const std::string_view json, const std::string_view key,
                          std::size_t& output) {
  const std::optional<std::string_view> value = jsonValueForKey(json, key);
  if (!value.has_value() || *value == "null") {
    return;
  }

  const std::string value_string{*value};
  char* end = nullptr;
  const double parsed = std::strtod(value_string.c_str(), &end);
  if (end != value_string.c_str() && *end == '\0' && std::isfinite(parsed) &&
      parsed >= 0.0) {
    output = static_cast<std::size_t>(parsed);
  }
}

[[nodiscard]] inline bool parseJsonUint64(const std::string_view json,
                                          const std::string_view key,
                                          std::uint64_t& output) {
  const std::optional<std::string_view> value = jsonValueForKey(json, key);
  if (!value.has_value() || *value == "null") {
    return false;
  }
  const std::string value_string{*value};
  char* end = nullptr;
  const unsigned long long parsed = std::strtoull(value_string.c_str(), &end, 10);
  if (end == value_string.c_str() || *end != '\0') {
    return false;
  }
  output = static_cast<std::uint64_t>(parsed);
  return true;
}

inline void parseJsonBool(const std::string_view json, const std::string_view key,
                          bool& output) {
  const std::optional<std::string_view> value = jsonValueForKey(json, key);
  if (!value.has_value()) {
    return;
  }
  if (*value == "true") {
    output = true;
    return;
  }
  if (*value == "false") {
    output = false;
  }
}

[[nodiscard]] inline TrajectoryPlannerStatus
parseTrajectoryPlannerStatusName(const std::string_view value) {
  if (value == trajectoryPlannerStatusName(TrajectoryPlannerStatus::kInvalidRoute)) {
    return TrajectoryPlannerStatus::kInvalidRoute;
  }
  if (value == trajectoryPlannerStatusName(TrajectoryPlannerStatus::kMissingGrid)) {
    return TrajectoryPlannerStatus::kMissingGrid;
  }
  if (value == trajectoryPlannerStatusName(TrajectoryPlannerStatus::kCorridorInvalid)) {
    return TrajectoryPlannerStatus::kCorridorInvalid;
  }
  if (value == trajectoryPlannerStatusName(
                   TrajectoryPlannerStatus::kTrajectoryOptimizerInvalid)) {
    return TrajectoryPlannerStatus::kTrajectoryOptimizerInvalid;
  }
  if (value ==
      trajectoryPlannerStatusName(TrajectoryPlannerStatus::kInvalidTrajectory)) {
    return TrajectoryPlannerStatus::kInvalidTrajectory;
  }
  return TrajectoryPlannerStatus::kOk;
}

[[nodiscard]] inline TrajectoryQuality
parseTrajectoryQualityName(const std::string_view value) {
  if (value == trajectoryQualityName(TrajectoryQuality::kBaseline)) {
    return TrajectoryQuality::kBaseline;
  }
  if (value == trajectoryQualityName(TrajectoryQuality::kRefined)) {
    return TrajectoryQuality::kRefined;
  }
  return TrajectoryQuality::kUnknown;
}

[[nodiscard]] inline SpeedConstraintType
parseSpeedConstraintTypeName(const std::string_view value) {
  if (value == speedConstraintTypeName(SpeedConstraintType::kArc)) {
    return SpeedConstraintType::kArc;
  }
  if (value == speedConstraintTypeName(SpeedConstraintType::kGoal)) {
    return SpeedConstraintType::kGoal;
  }
  return SpeedConstraintType::kNone;
}

[[nodiscard]] inline std::string
speedProfileTopConstraintPrefix(const std::size_t index) {
  return "speed_profile_top" + std::to_string(index + 1U) + "_";
}

inline std::string
speedProfileConstraintDiagnosticsJsonFieldsImpl(const TrajectoryPlannerStats& stats) {
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"speed_profile_top_constraint_count\":"
         << stats.top_speed_constraints.size();
  for (std::size_t i = 0U; i < stats.top_speed_constraints.size(); ++i) {
    const SpeedProfileConstraintDiagnostic& constraint = stats.top_speed_constraints[i];
    const std::string prefix = speedProfileTopConstraintPrefix(i);
    appendJsonSize(stream, prefix + "sample_index", constraint.sample_index);
    appendJsonNumber(stream, prefix + "s_m", constraint.s_m);
    appendJsonNumber(stream, prefix + "radius_m", constraint.radius_m);
    appendJsonNumber(stream, prefix + "curvature_1pm", constraint.curvature_1pm);
    appendJsonNumber(stream, prefix + "speed_limit_mps", constraint.speed_limit_mps);
    appendJsonNumber(stream, prefix + "profiled_limit_mps",
                     constraint.profiled_limit_mps);
    appendJsonString(stream, prefix + "source",
                     speedConstraintTypeName(constraint.source));
    appendJsonBool(stream, prefix + "isolated_curvature_spike",
                   constraint.isolated_curvature_spike);
  }
  appendJsonSize(stream, "isolated_curvature_spike_candidates",
                 stats.isolated_curvature_spike_candidates);
  appendJsonSize(stream, "isolated_curvature_spikes_smoothed_geometry",
                 stats.isolated_curvature_spikes_smoothed_geometry);
  appendJsonSize(stream, "isolated_curvature_spikes_smoothed_speed_profile",
                 stats.isolated_curvature_spikes_smoothed_speed_profile);
  appendJsonNumber(stream, "isolated_curvature_spike_max_before_1pm",
                   stats.isolated_curvature_spike_max_before_1pm);
  appendJsonNumber(stream, "isolated_curvature_spike_max_after_1pm",
                   stats.isolated_curvature_spike_max_after_1pm);
  return stream.str();
}

} // namespace trajectory_diagnostics_io_detail
} // namespace drone_city_nav
