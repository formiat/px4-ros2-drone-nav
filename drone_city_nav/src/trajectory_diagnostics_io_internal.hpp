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

inline void writeJsonEscapedString(std::ostream& stream, const std::string_view value) {
  constexpr std::array<char, 16U> kHexDigits{'0', '1', '2', '3', '4', '5', '6', '7',
                                             '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};
  for (const char character : value) {
    switch (character) {
      case '"':
        stream << "\\\"";
        break;
      case '\\':
        stream << "\\\\";
        break;
      case '\b':
        stream << "\\b";
        break;
      case '\f':
        stream << "\\f";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(character) < 0x20U) {
          const auto value_byte = static_cast<unsigned char>(character);
          stream << "\\u00" << kHexDigits[value_byte >> 4U]
                 << kHexDigits[value_byte & 0x0FU];
        } else {
          stream << character;
        }
        break;
    }
  }
}

inline void appendJsonString(std::ostream& stream, const std::string_view key,
                             const std::string_view value) {
  stream << ",\"" << key << "\":\"";
  writeJsonEscapedString(stream, value);
  stream << "\"";
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
    for (std::size_t string_end = value_begin + 1U; string_end < json.size();
         ++string_end) {
      if (json[string_end] == '\\') {
        ++string_end;
        continue;
      }
      if (json[string_end] == '"') {
        return json.substr(value_begin + 1U, string_end - value_begin - 1U);
      }
    }
    return std::nullopt;
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

[[nodiscard]] inline std::string decodeJsonStringValue(const std::string_view value) {
  const auto hexValue = [](const char character) -> int {
    if (character >= '0' && character <= '9') {
      return character - '0';
    }
    if (character >= 'a' && character <= 'f') {
      return character - 'a' + 10;
    }
    if (character >= 'A' && character <= 'F') {
      return character - 'A' + 10;
    }
    return -1;
  };
  std::string output;
  output.reserve(value.size());
  for (std::size_t index = 0U; index < value.size(); ++index) {
    const char character = value[index];
    if (character != '\\' || index + 1U >= value.size()) {
      output.push_back(character);
      continue;
    }

    ++index;
    switch (value[index]) {
      case '"':
        output.push_back('"');
        break;
      case '\\':
        output.push_back('\\');
        break;
      case '/':
        output.push_back('/');
        break;
      case 'b':
        output.push_back('\b');
        break;
      case 'f':
        output.push_back('\f');
        break;
      case 'n':
        output.push_back('\n');
        break;
      case 'r':
        output.push_back('\r');
        break;
      case 't':
        output.push_back('\t');
        break;
      case 'u':
        if (index + 4U < value.size() && value[index + 1U] == '0' &&
            value[index + 2U] == '0') {
          const int high = hexValue(value[index + 3U]);
          const int low = hexValue(value[index + 4U]);
          if (high >= 0 && low >= 0) {
            output.push_back(static_cast<char>((high << 4U) | low));
            index += 4U;
            break;
          }
        }
        output.push_back('u');
        break;
      default:
        output.push_back(value[index]);
        break;
    }
  }
  return output;
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
  if (value == speedConstraintTypeName(SpeedConstraintType::kVerticalProfile)) {
    return SpeedConstraintType::kVerticalProfile;
  }
  if (value == speedConstraintTypeName(SpeedConstraintType::kVerticalTrackability)) {
    return SpeedConstraintType::kVerticalTrackability;
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
  appendJsonNumber(stream, "isolated_curvature_spike_max_before_1pm",
                   stats.isolated_curvature_spike_max_before_1pm);
  appendJsonNumber(stream, "isolated_curvature_spike_max_after_1pm",
                   stats.isolated_curvature_spike_max_after_1pm);
  return stream.str();
}

[[nodiscard]] inline std::string knownPassageDiagnosticPrefix(const std::size_t index) {
  return "known_passage_diag" + std::to_string(index) + "_";
}

[[nodiscard]] inline KnownPassageValidationReason
parseKnownPassageValidationReasonName(const std::string_view value) {
  if (value ==
      knownPassageValidationReasonName(KnownPassageValidationReason::kDisabled)) {
    return KnownPassageValidationReason::kDisabled;
  }
  if (value == knownPassageValidationReasonName(KnownPassageValidationReason::kNoMap)) {
    return KnownPassageValidationReason::kNoMap;
  }
  if (value == knownPassageValidationReasonName(
                   KnownPassageValidationReason::kInvalidTrajectory)) {
    return KnownPassageValidationReason::kInvalidTrajectory;
  }
  if (value ==
      knownPassageValidationReasonName(KnownPassageValidationReason::kMatchedOpening)) {
    return KnownPassageValidationReason::kMatchedOpening;
  }
  if (value == knownPassageValidationReasonName(
                   KnownPassageValidationReason::kStructureWithoutOpening)) {
    return KnownPassageValidationReason::kStructureWithoutOpening;
  }
  if (value == knownPassageValidationReasonName(
                   KnownPassageValidationReason::kOpeningVolumeMiss)) {
    return KnownPassageValidationReason::kOpeningVolumeMiss;
  }
  return KnownPassageValidationReason::kNoStructureIntersection;
}

inline std::string
knownPassageValidationDiagnosticsJsonFieldsImpl(const TrajectoryPlannerStats& stats) {
  const KnownPassageValidationSummary& validation = stats.known_passage_validation;
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"known_passage_validation_enabled\":"
         << (validation.enabled ? "true" : "false");
  appendJsonBool(stream, "known_passage_validation_valid", validation.valid);
  appendJsonSize(stream, "known_passage_structures_checked",
                 validation.structures_checked);
  appendJsonSize(stream, "known_passage_structures_intersected",
                 validation.structures_intersected);
  appendJsonSize(stream, "known_passage_opening_matches", validation.opening_matches);
  appendJsonSize(stream, "known_passage_violations", validation.violations);
  appendJsonString(stream, "known_passage_validation_reason",
                   knownPassageValidationReasonName(validation.worst_reason));
  appendJsonSize(stream, "known_passage_diag_count", validation.diagnostics.size());
  for (std::size_t i = 0U; i < validation.diagnostics.size(); ++i) {
    const KnownPassageValidationSpan& diagnostic = validation.diagnostics[i];
    const std::string prefix = knownPassageDiagnosticPrefix(i);
    appendJsonString(stream, prefix + "structure_id", diagnostic.structure_id);
    appendJsonString(stream, prefix + "opening_id", diagnostic.opening_id);
    appendJsonNumber(stream, prefix + "entry_s_m", diagnostic.entry_s_m);
    appendJsonNumber(stream, prefix + "exit_s_m", diagnostic.exit_s_m);
    appendJsonNumber(stream, prefix + "overlap_m", diagnostic.overlap_m);
    appendJsonNumber(stream, prefix + "clearance_m", diagnostic.clearance_m);
    appendJsonString(stream, prefix + "reason",
                     knownPassageValidationReasonName(diagnostic.reason));
    appendJsonBool(stream, prefix + "valid", diagnostic.valid);
  }
  return stream.str();
}

[[nodiscard]] inline std::string
verticalProfileDiagnosticPrefix(const std::size_t index) {
  return "vertical_profile_diag" + std::to_string(index) + "_";
}

inline std::string
verticalProfileDiagnosticsJsonFieldsImpl(const TrajectoryPlannerStats& stats) {
  const VerticalProfileStats& profile = stats.vertical_profile;
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"vertical_profile_enabled\":" << (profile.enabled ? "true" : "false");
  appendJsonBool(stream, "vertical_profile_active", profile.active);
  appendJsonBool(stream, "vertical_profile_applied", profile.applied);
  appendJsonBool(stream, "vertical_profile_valid", profile.valid);
  appendJsonSize(stream, "vertical_profile_passages_matched", profile.passages_matched);
  appendJsonSize(stream, "vertical_profile_passages_profiled",
                 profile.passages_profiled);
  appendJsonSize(stream, "vertical_profile_infeasible_count", profile.infeasible_count);
  appendJsonNumber(stream, "vertical_profile_min_z_m", profile.min_z_m);
  appendJsonNumber(stream, "vertical_profile_max_z_m", profile.max_z_m);
  appendJsonNumber(stream, "vertical_profile_max_abs_dz_ds", profile.max_abs_dz_ds);
  appendJsonNumber(stream, "vertical_profile_max_abs_d2z_ds2", profile.max_abs_d2z_ds2);
  appendJsonNumber(stream, "vertical_profile_max_abs_d3z_ds3", profile.max_abs_d3z_ds3);
  appendJsonNumber(stream, "vertical_profile_min_vertical_speed_cap_mps",
                   profile.min_vertical_speed_cap_mps);
  appendJsonSize(stream, "vertical_profile_diag_count", profile.diagnostics.size());
  for (std::size_t i = 0U; i < profile.diagnostics.size(); ++i) {
    const VerticalProfilePassageDiagnostic& diagnostic = profile.diagnostics[i];
    const std::string prefix = verticalProfileDiagnosticPrefix(i);
    appendJsonString(stream, prefix + "structure_id", diagnostic.structure_id);
    appendJsonString(stream, prefix + "opening_id", diagnostic.opening_id);
    appendJsonNumber(stream, prefix + "entry_s_m", diagnostic.entry_s_m);
    appendJsonNumber(stream, prefix + "exit_s_m", diagnostic.exit_s_m);
    appendJsonNumber(stream, prefix + "approach_start_s_m",
                     diagnostic.approach_start_s_m);
    appendJsonNumber(stream, prefix + "gate_hold_start_s_m",
                     diagnostic.gate_hold_start_s_m);
    appendJsonNumber(stream, prefix + "exit_end_s_m", diagnostic.exit_end_s_m);
    appendJsonNumber(stream, prefix + "gate_z_m", diagnostic.gate_z_m);
    appendJsonNumber(stream, prefix + "min_z_m", diagnostic.min_z_m);
    appendJsonNumber(stream, prefix + "max_z_m", diagnostic.max_z_m);
    appendJsonString(stream, prefix + "reason", diagnostic.reason);
    appendJsonBool(stream, prefix + "valid", diagnostic.valid);
  }
  return stream.str();
}

[[nodiscard]] inline std::string
passageInsertionDiagnosticPrefix(const std::size_t index) {
  return "passage_insertion_diag" + std::to_string(index) + "_";
}

[[nodiscard]] inline PassageInsertionRejectReason
parsePassageInsertionRejectReasonName(const std::string_view value) {
  if (value == passageInsertionRejectReasonName(PassageInsertionRejectReason::kNone)) {
    return PassageInsertionRejectReason::kNone;
  }
  if (value ==
      passageInsertionRejectReasonName(PassageInsertionRejectReason::kDisabled)) {
    return PassageInsertionRejectReason::kDisabled;
  }
  if (value == passageInsertionRejectReasonName(PassageInsertionRejectReason::kNoMap)) {
    return PassageInsertionRejectReason::kNoMap;
  }
  if (value ==
      passageInsertionRejectReasonName(PassageInsertionRejectReason::kInvalidInput)) {
    return PassageInsertionRejectReason::kInvalidInput;
  }
  if (value ==
      passageInsertionRejectReasonName(PassageInsertionRejectReason::kNoRepairNeeded)) {
    return PassageInsertionRejectReason::kNoRepairNeeded;
  }
  if (value ==
      passageInsertionRejectReasonName(PassageInsertionRejectReason::kNoCandidate)) {
    return PassageInsertionRejectReason::kNoCandidate;
  }
  if (value == passageInsertionRejectReasonName(
                   PassageInsertionRejectReason::kTooManyCandidates)) {
    return PassageInsertionRejectReason::kTooManyCandidates;
  }
  if (value == passageInsertionRejectReasonName(
                   PassageInsertionRejectReason::kInvalidOpeningFrame)) {
    return PassageInsertionRejectReason::kInvalidOpeningFrame;
  }
  if (value == passageInsertionRejectReasonName(
                   PassageInsertionRejectReason::kExcessiveLateralShift)) {
    return PassageInsertionRejectReason::kExcessiveLateralShift;
  }
  if (value == passageInsertionRejectReasonName(
                   PassageInsertionRejectReason::kInvalidGeometry)) {
    return PassageInsertionRejectReason::kInvalidGeometry;
  }
  if (value ==
      passageInsertionRejectReasonName(PassageInsertionRejectReason::kNonTraversable)) {
    return PassageInsertionRejectReason::kNonTraversable;
  }
  if (value == passageInsertionRejectReasonName(
                   PassageInsertionRejectReason::kEndpointMismatch)) {
    return PassageInsertionRejectReason::kEndpointMismatch;
  }
  if (value ==
      passageInsertionRejectReasonName(PassageInsertionRejectReason::kJoinTangent)) {
    return PassageInsertionRejectReason::kJoinTangent;
  }
  if (value ==
      passageInsertionRejectReasonName(PassageInsertionRejectReason::kJoinCurvature)) {
    return PassageInsertionRejectReason::kJoinCurvature;
  }
  if (value ==
      passageInsertionRejectReasonName(PassageInsertionRejectReason::kInsertedRadius)) {
    return PassageInsertionRejectReason::kInsertedRadius;
  }
  if (value == passageInsertionRejectReasonName(
                   PassageInsertionRejectReason::kValidationNotImproved)) {
    return PassageInsertionRejectReason::kValidationNotImproved;
  }
  return PassageInsertionRejectReason::kNone;
}

inline std::string
passageInsertionDiagnosticsJsonFieldsImpl(const TrajectoryPlannerStats& stats) {
  const PassageInsertionStats& insertion = stats.passage_insertion;
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream << "\"passage_insertion_enabled\":" << (insertion.enabled ? "true" : "false");
  appendJsonBool(stream, "passage_insertion_applied", insertion.applied);
  appendJsonSize(stream, "passage_insertion_candidates", insertion.candidates);
  appendJsonSize(stream, "passage_insertion_inserted_count", insertion.inserted_count);
  appendJsonSize(stream, "passage_insertion_rejected_join", insertion.rejected_join);
  appendJsonSize(stream, "passage_insertion_rejected_traversability",
                 insertion.rejected_traversability);
  appendJsonSize(stream, "passage_insertion_rejected_validation",
                 insertion.rejected_validation);
  appendJsonSize(stream, "passage_insertion_rejected_geometry",
                 insertion.rejected_geometry);
  appendJsonSize(stream, "passage_insertion_diagnostics_dropped",
                 insertion.diagnostics_dropped);
  appendJsonString(stream, "passage_insertion_reason",
                   passageInsertionRejectReasonName(insertion.final_reason));
  appendJsonNumber(stream, "passage_insertion_duration_ms",
                   stats.passage_insertion_duration_ms);
  appendJsonSize(stream, "passage_insertion_diag_count", insertion.diagnostics.size());
  for (std::size_t i = 0U; i < insertion.diagnostics.size(); ++i) {
    const PassageInsertionDiagnostic& diagnostic = insertion.diagnostics[i];
    const std::string prefix = passageInsertionDiagnosticPrefix(i);
    appendJsonString(stream, prefix + "structure_id", diagnostic.structure_id);
    appendJsonString(stream, prefix + "opening_id", diagnostic.opening_id);
    appendJsonNumber(stream, prefix + "anchor_s_m", diagnostic.anchor_s_m);
    appendJsonNumber(stream, prefix + "entry_s_m", diagnostic.entry_s_m);
    appendJsonNumber(stream, prefix + "exit_s_m", diagnostic.exit_s_m);
    appendJsonNumber(stream, prefix + "reconnect_s_m", diagnostic.reconnect_s_m);
    appendJsonNumber(stream, prefix + "lateral_miss_before_m",
                     diagnostic.lateral_miss_before_m);
    appendJsonNumber(stream, prefix + "lateral_miss_after_m",
                     diagnostic.lateral_miss_after_m);
    appendJsonNumber(stream, prefix + "join_tangent_delta_before_rad",
                     diagnostic.join_tangent_delta_before_rad);
    appendJsonNumber(stream, prefix + "join_tangent_delta_after_rad",
                     diagnostic.join_tangent_delta_after_rad);
    appendJsonNumber(stream, prefix + "join_curvature_jump_before_1pm",
                     diagnostic.join_curvature_jump_before_1pm);
    appendJsonNumber(stream, prefix + "join_curvature_jump_after_1pm",
                     diagnostic.join_curvature_jump_after_1pm);
    appendJsonNumber(stream, prefix + "min_inserted_radius_m",
                     diagnostic.min_inserted_radius_m);
    appendJsonString(stream, prefix + "reason",
                     passageInsertionRejectReasonName(diagnostic.reason));
    appendJsonBool(stream, prefix + "accepted", diagnostic.accepted);
  }
  return stream.str();
}

} // namespace trajectory_diagnostics_io_detail
} // namespace drone_city_nav
