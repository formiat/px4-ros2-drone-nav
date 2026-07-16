#include "drone_city_nav/obstacle_memory_provenance_ros.hpp"

#include "drone_city_nav/msg/obstacle_memory_cell_provenance.hpp"
#include "drone_city_nav/msg/obstacle_memory_hit_observation.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>
#include <sstream>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000LL;

[[nodiscard]] bool finitePoint(const geometry_msgs::msg::Point& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool finiteVector(const geometry_msgs::msg::Vector3& vector) noexcept {
  return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
}

[[nodiscard]] std::optional<int> finiteFloorToInt(const double value) noexcept {
  if (!std::isfinite(value)) {
    return std::nullopt;
  }
  const double floored = std::floor(value);
  if (!std::isfinite(floored) ||
      floored < static_cast<double>(std::numeric_limits<int>::min()) ||
      floored > static_cast<double>(std::numeric_limits<int>::max())) {
    return std::nullopt;
  }
  return static_cast<int>(floored);
}

[[nodiscard]] std::optional<std::int64_t>
checkedStampNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  if (stamp.sec < 0 ||
      stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond)) {
    return std::nullopt;
  }
  const std::int64_t seconds = static_cast<std::int64_t>(stamp.sec);
  if (seconds > std::numeric_limits<std::int64_t>::max() / kNanosecondsPerSecond) {
    return std::nullopt;
  }
  const std::int64_t value =
      seconds * kNanosecondsPerSecond + static_cast<std::int64_t>(stamp.nanosec);
  if (value == 0) {
    return std::nullopt;
  }
  return value;
}

[[nodiscard]] builtin_interfaces::msg::Time
stampFromNanoseconds(const std::int64_t stamp_ns, const bool valid) noexcept {
  builtin_interfaces::msg::Time stamp;
  if (!valid || stamp_ns <= 0) {
    return stamp;
  }
  stamp.sec = static_cast<std::int32_t>(stamp_ns / kNanosecondsPerSecond);
  stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % kNanosecondsPerSecond);
  return stamp;
}

[[nodiscard]] bool sameStamp(const builtin_interfaces::msg::Time& lhs,
                             const builtin_interfaces::msg::Time& rhs) noexcept {
  return lhs.sec == rhs.sec && lhs.nanosec == rhs.nanosec;
}

[[nodiscard]] bool samePose(const geometry_msgs::msg::Pose& lhs,
                            const geometry_msgs::msg::Pose& rhs) noexcept {
  return lhs.position.x == rhs.position.x && lhs.position.y == rhs.position.y &&
         lhs.position.z == rhs.position.z && lhs.orientation.x == rhs.orientation.x &&
         lhs.orientation.y == rhs.orientation.y &&
         lhs.orientation.z == rhs.orientation.z &&
         lhs.orientation.w == rhs.orientation.w;
}

[[nodiscard]] bool sameGridInfo(const nav_msgs::msg::MapMetaData& lhs,
                                const nav_msgs::msg::MapMetaData& rhs) noexcept {
  return sameStamp(lhs.map_load_time, rhs.map_load_time) &&
         lhs.resolution == rhs.resolution && lhs.width == rhs.width &&
         lhs.height == rhs.height && samePose(lhs.origin, rhs.origin);
}

[[nodiscard]] bool validGridInfo(const nav_msgs::msg::MapMetaData& info) noexcept {
  const double quaternion_norm =
      std::hypot(std::hypot(info.origin.orientation.x, info.origin.orientation.y),
                 std::hypot(info.origin.orientation.z, info.origin.orientation.w));
  return info.width > 0U && info.height > 0U &&
         info.width <= static_cast<std::uint32_t>(std::numeric_limits<int>::max()) &&
         info.height <= static_cast<std::uint32_t>(std::numeric_limits<int>::max()) &&
         std::isfinite(info.resolution) && info.resolution > 0.0F &&
         finitePoint(info.origin.position) &&
         std::isfinite(info.origin.orientation.x) &&
         std::isfinite(info.origin.orientation.y) &&
         std::isfinite(info.origin.orientation.z) &&
         std::isfinite(info.origin.orientation.w) && std::isfinite(quaternion_norm) &&
         std::abs(quaternion_norm - 1.0) <= 1.0e-6;
}

[[nodiscard]] std::uint8_t classificationToMessage(
    const KnownStaticLidarHitClassification classification) noexcept {
  using Observation = msg::ObstacleMemoryHitObservation;
  switch (classification) {
    case KnownStaticLidarHitClassification::kExpectedStatic:
      return Observation::CLASSIFICATION_EXPECTED_STATIC;
    case KnownStaticLidarHitClassification::kUnexpected:
      return Observation::CLASSIFICATION_UNEXPECTED;
    case KnownStaticLidarHitClassification::kAmbiguous:
      return Observation::CLASSIFICATION_AMBIGUOUS;
  }
  return Observation::CLASSIFICATION_AMBIGUOUS;
}

[[nodiscard]] std::optional<KnownStaticLidarHitClassification>
classificationFromMessage(const std::uint8_t value) noexcept {
  using Observation = msg::ObstacleMemoryHitObservation;
  switch (value) {
    case Observation::CLASSIFICATION_EXPECTED_STATIC:
      return KnownStaticLidarHitClassification::kExpectedStatic;
    case Observation::CLASSIFICATION_UNEXPECTED:
      return KnownStaticLidarHitClassification::kUnexpected;
    case Observation::CLASSIFICATION_AMBIGUOUS:
      return KnownStaticLidarHitClassification::kAmbiguous;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] std::uint8_t
partKindToMessage(const KnownPassageSolidPartKind kind) noexcept {
  using Observation = msg::ObstacleMemoryHitObservation;
  switch (kind) {
    case KnownPassageSolidPartKind::kLeft:
      return Observation::KNOWN_PART_LEFT;
    case KnownPassageSolidPartKind::kRight:
      return Observation::KNOWN_PART_RIGHT;
    case KnownPassageSolidPartKind::kLower:
      return Observation::KNOWN_PART_LOWER;
    case KnownPassageSolidPartKind::kUpper:
      return Observation::KNOWN_PART_UPPER;
  }
  return Observation::KNOWN_PART_LEFT;
}

[[nodiscard]] std::optional<KnownPassageSolidPartKind>
partKindFromMessage(const std::uint8_t value) noexcept {
  using Observation = msg::ObstacleMemoryHitObservation;
  switch (value) {
    case Observation::KNOWN_PART_LEFT:
      return KnownPassageSolidPartKind::kLeft;
    case Observation::KNOWN_PART_RIGHT:
      return KnownPassageSolidPartKind::kRight;
    case Observation::KNOWN_PART_LOWER:
      return KnownPassageSolidPartKind::kLower;
    case Observation::KNOWN_PART_UPPER:
      return KnownPassageSolidPartKind::kUpper;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] std::uint8_t
ingestionActionToMessage(const LidarIngestionAction action) noexcept {
  using Observation = msg::ObstacleMemoryHitObservation;
  switch (action) {
    case LidarIngestionAction::kIntegrateFreeAndHit:
      return Observation::INGESTION_ACTION_FREE_AND_HIT;
    case LidarIngestionAction::kIntegrateFreeOnly:
      return Observation::INGESTION_ACTION_FREE_ONLY;
    case LidarIngestionAction::kSuppressAllUpdates:
      return Observation::INGESTION_ACTION_SUPPRESS_ALL;
  }
  return Observation::INGESTION_ACTION_SUPPRESS_ALL;
}

[[nodiscard]] std::optional<LidarIngestionAction>
ingestionActionFromMessage(const std::uint8_t value) noexcept {
  using Observation = msg::ObstacleMemoryHitObservation;
  switch (value) {
    case Observation::INGESTION_ACTION_FREE_AND_HIT:
      return LidarIngestionAction::kIntegrateFreeAndHit;
    case Observation::INGESTION_ACTION_FREE_ONLY:
      return LidarIngestionAction::kIntegrateFreeOnly;
    case Observation::INGESTION_ACTION_SUPPRESS_ALL:
      return LidarIngestionAction::kSuppressAllUpdates;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] std::uint8_t
ingestionReasonToMessage(const LidarIngestionReason reason) noexcept {
  using Observation = msg::ObstacleMemoryHitObservation;
  switch (reason) {
    case LidarIngestionReason::kNoExpectedSurface:
      return Observation::INGESTION_REASON_NO_EXPECTED_SURFACE;
    case LidarIngestionReason::kObstacleBeforeExpectedSurface:
      return Observation::INGESTION_REASON_OBSTACLE_BEFORE_EXPECTED_SURFACE;
    case LidarIngestionReason::kExpectedKnownStatic:
      return Observation::INGESTION_REASON_EXPECTED_KNOWN_STATIC;
    case LidarIngestionReason::kUnexpectedKnownStatic:
      return Observation::INGESTION_REASON_UNEXPECTED_KNOWN_STATIC;
    case LidarIngestionReason::kAmbiguousKnownStatic:
      return Observation::INGESTION_REASON_AMBIGUOUS_KNOWN_STATIC;
    case LidarIngestionReason::kExpectedGround:
      return Observation::INGESTION_REASON_EXPECTED_GROUND;
    case LidarIngestionReason::kAmbiguousGround:
      return Observation::INGESTION_REASON_AMBIGUOUS_GROUND;
    case LidarIngestionReason::kTiedExpectedSurfaces:
      return Observation::INGESTION_REASON_TIED_EXPECTED_SURFACES;
    case LidarIngestionReason::kClassificationUnavailable:
      return Observation::INGESTION_REASON_CLASSIFICATION_UNAVAILABLE;
  }
  return Observation::INGESTION_REASON_CLASSIFICATION_UNAVAILABLE;
}

[[nodiscard]] std::optional<LidarIngestionReason>
ingestionReasonFromMessage(const std::uint8_t value) noexcept {
  using Observation = msg::ObstacleMemoryHitObservation;
  switch (value) {
    case Observation::INGESTION_REASON_NO_EXPECTED_SURFACE:
      return LidarIngestionReason::kNoExpectedSurface;
    case Observation::INGESTION_REASON_OBSTACLE_BEFORE_EXPECTED_SURFACE:
      return LidarIngestionReason::kObstacleBeforeExpectedSurface;
    case Observation::INGESTION_REASON_EXPECTED_KNOWN_STATIC:
      return LidarIngestionReason::kExpectedKnownStatic;
    case Observation::INGESTION_REASON_UNEXPECTED_KNOWN_STATIC:
      return LidarIngestionReason::kUnexpectedKnownStatic;
    case Observation::INGESTION_REASON_AMBIGUOUS_KNOWN_STATIC:
      return LidarIngestionReason::kAmbiguousKnownStatic;
    case Observation::INGESTION_REASON_EXPECTED_GROUND:
      return LidarIngestionReason::kExpectedGround;
    case Observation::INGESTION_REASON_AMBIGUOUS_GROUND:
      return LidarIngestionReason::kAmbiguousGround;
    case Observation::INGESTION_REASON_TIED_EXPECTED_SURFACES:
      return LidarIngestionReason::kTiedExpectedSurfaces;
    case Observation::INGESTION_REASON_CLASSIFICATION_UNAVAILABLE:
      return LidarIngestionReason::kClassificationUnavailable;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] std::uint8_t
expectedSurfaceToMessage(const LidarExpectedSurfaceKind surface) noexcept {
  using Observation = msg::ObstacleMemoryHitObservation;
  switch (surface) {
    case LidarExpectedSurfaceKind::kNone:
      return Observation::EXPECTED_SURFACE_NONE;
    case LidarExpectedSurfaceKind::kKnownStatic:
      return Observation::EXPECTED_SURFACE_KNOWN_STATIC;
    case LidarExpectedSurfaceKind::kGround:
      return Observation::EXPECTED_SURFACE_GROUND;
    case LidarExpectedSurfaceKind::kTied:
      return Observation::EXPECTED_SURFACE_TIED;
  }
  return Observation::EXPECTED_SURFACE_NONE;
}

[[nodiscard]] std::optional<LidarExpectedSurfaceKind>
expectedSurfaceFromMessage(const std::uint8_t value) noexcept {
  using Observation = msg::ObstacleMemoryHitObservation;
  switch (value) {
    case Observation::EXPECTED_SURFACE_NONE:
      return LidarExpectedSurfaceKind::kNone;
    case Observation::EXPECTED_SURFACE_KNOWN_STATIC:
      return LidarExpectedSurfaceKind::kKnownStatic;
    case Observation::EXPECTED_SURFACE_GROUND:
      return LidarExpectedSurfaceKind::kGround;
    case Observation::EXPECTED_SURFACE_TIED:
      return LidarExpectedSurfaceKind::kTied;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] bool validAcceptedIngestionDecision(
    const LidarIngestionAction action, const LidarIngestionReason reason,
    const LidarExpectedSurfaceKind surface, const double expected_range_m,
    const double range_delta_m) noexcept {
  if (action != LidarIngestionAction::kIntegrateFreeAndHit) {
    return false;
  }

  const bool no_expected_surface = surface == LidarExpectedSurfaceKind::kNone &&
                                   std::isnan(expected_range_m) &&
                                   std::isnan(range_delta_m);
  const bool known_static_surface = surface == LidarExpectedSurfaceKind::kKnownStatic &&
                                    std::isfinite(expected_range_m) &&
                                    expected_range_m > 0.0 &&
                                    std::isfinite(range_delta_m);
  const bool ground_surface = surface == LidarExpectedSurfaceKind::kGround &&
                              std::isfinite(expected_range_m) &&
                              expected_range_m > 0.0 && std::isfinite(range_delta_m);

  switch (reason) {
    case LidarIngestionReason::kNoExpectedSurface:
    case LidarIngestionReason::kClassificationUnavailable:
      return no_expected_surface;
    case LidarIngestionReason::kObstacleBeforeExpectedSurface:
      return (known_static_surface || ground_surface) && range_delta_m < 0.0;
    case LidarIngestionReason::kUnexpectedKnownStatic:
    case LidarIngestionReason::kAmbiguousKnownStatic:
      return no_expected_surface || known_static_surface;
    case LidarIngestionReason::kExpectedKnownStatic:
    case LidarIngestionReason::kExpectedGround:
    case LidarIngestionReason::kAmbiguousGround:
    case LidarIngestionReason::kTiedExpectedSurfaces:
      return false;
  }
  return false;
}

[[nodiscard]] msg::ObstacleMemoryHitObservation
observationToMessage(const AcceptedObstacleMemoryHit& hit) {
  msg::ObstacleMemoryHitObservation message;
  const LidarBeamObservation& beam = hit.beam;
  message.beam_index = static_cast<std::uint64_t>(beam.beam_index);
  message.acquisition_stamp =
      stampFromNanoseconds(beam.acquisition_stamp_ns, beam.acquisition_stamp_valid);
  message.acquisition_stamp_valid = beam.acquisition_stamp_valid;
  message.receive_stamp =
      stampFromNanoseconds(beam.receive_stamp_ns, beam.receive_stamp_valid);
  message.receive_stamp_valid = beam.receive_stamp_valid;
  message.ray_origin_map_m.x = beam.projection.ray_origin_map_m.x;
  message.ray_origin_map_m.y = beam.projection.ray_origin_map_m.y;
  message.ray_origin_map_m.z = beam.projection.ray_origin_map_m.z;
  message.ray_direction_map.x = beam.projection.ray_direction_map.x;
  message.ray_direction_map.y = beam.projection.ray_direction_map.y;
  message.ray_direction_map.z = beam.projection.ray_direction_map.z;
  message.endpoint_map_m.x = beam.projection.endpoint.x;
  message.endpoint_map_m.y = beam.projection.endpoint.y;
  message.endpoint_map_m.z = beam.projection.endpoint_xyz_valid
                                 ? beam.projection.endpoint_map_m.z
                                 : std::numeric_limits<double>::quiet_NaN();
  message.endpoint_xyz_valid = beam.projection.endpoint_xyz_valid;
  message.measured_range_m = beam.measured_range_m;
  message.source_attitude_valid = beam.source_attitude_valid;
  message.source_roll_rad = beam.source_roll_rad;
  message.source_pitch_rad = beam.source_pitch_rad;
  message.source_tilt_rad = beam.source_tilt_rad;
  message.attitude_compensation_applied = beam.projection.attitude_compensation_applied;
  message.applied_roll_rad = beam.projection.applied_roll_rad;
  message.applied_pitch_rad = beam.projection.applied_pitch_rad;
  message.applied_tilt_rad = beam.projection.applied_tilt_rad;
  message.classifier_applied = hit.known_static.classifier_applied;
  message.classification = classificationToMessage(hit.known_static.classification);
  message.volume_matched = hit.known_static.volume_matched;
  message.confident_face_interior = hit.known_static.confident_face_interior;
  message.known_part_valid = hit.known_static.part_kind_valid;
  message.known_part = partKindToMessage(hit.known_static.part_kind);
  message.structure_id = hit.known_static.structure_id;
  message.opening_id = hit.known_static.opening_id;
  message.part_id = hit.known_static.part_id;
  message.expected_range_m = hit.known_static.expected_range_m;
  message.range_delta_m = hit.known_static.range_delta_m;
  message.ingestion_action = ingestionActionToMessage(hit.ingestion_decision.action);
  message.ingestion_reason = ingestionReasonToMessage(hit.ingestion_decision.reason);
  message.ingestion_expected_surface =
      expectedSurfaceToMessage(hit.ingestion_decision.expected_surface);
  message.ingestion_expected_range_m = hit.ingestion_decision.expected_range_m;
  message.ingestion_range_delta_m = hit.ingestion_decision.range_delta_m;
  return message;
}

[[nodiscard]] std::optional<AcceptedObstacleMemoryHit>
observationFromMessage(const msg::ObstacleMemoryHitObservation& message) {
  const auto classification = classificationFromMessage(message.classification);
  const auto part_kind = partKindFromMessage(message.known_part);
  const auto ingestion_action = ingestionActionFromMessage(message.ingestion_action);
  const auto ingestion_reason = ingestionReasonFromMessage(message.ingestion_reason);
  const auto ingestion_surface =
      expectedSurfaceFromMessage(message.ingestion_expected_surface);
  const auto acquisition_stamp = checkedStampNanoseconds(message.acquisition_stamp);
  const auto receive_stamp = checkedStampNanoseconds(message.receive_stamp);
  const bool acquisition_stamp_zero =
      message.acquisition_stamp.sec == 0 && message.acquisition_stamp.nanosec == 0U;
  const bool receive_stamp_zero =
      message.receive_stamp.sec == 0 && message.receive_stamp.nanosec == 0U;
  const bool source_attitude_values_valid =
      message.source_attitude_valid ? std::isfinite(message.source_roll_rad) &&
                                          std::isfinite(message.source_pitch_rad) &&
                                          std::isfinite(message.source_tilt_rad)
                                    : std::isnan(message.source_roll_rad) &&
                                          std::isnan(message.source_pitch_rad) &&
                                          std::isnan(message.source_tilt_rad);
  const bool applied_attitude_values_valid =
      message.attitude_compensation_applied
          ? std::isfinite(message.applied_roll_rad) &&
                std::isfinite(message.applied_pitch_rad) &&
                std::isfinite(message.applied_tilt_rad)
          : message.applied_roll_rad == 0.0 && message.applied_pitch_rad == 0.0 &&
                message.applied_tilt_rad == 0.0;
  const bool ingestion_decision_valid =
      ingestion_action.has_value() && ingestion_reason.has_value() &&
      ingestion_surface.has_value() &&
      validAcceptedIngestionDecision(
          ingestion_action.value_or(LidarIngestionAction::kSuppressAllUpdates),
          ingestion_reason.value_or(LidarIngestionReason::kClassificationUnavailable),
          ingestion_surface.value_or(LidarExpectedSurfaceKind::kNone),
          message.ingestion_expected_range_m, message.ingestion_range_delta_m);
  if (!classification.has_value() || !part_kind.has_value() ||
      !ingestion_decision_valid ||
      message.beam_index > std::numeric_limits<std::size_t>::max() ||
      !finitePoint(message.ray_origin_map_m) ||
      !finiteVector(message.ray_direction_map) ||
      !std::isfinite(message.measured_range_m) || !(message.measured_range_m > 0.0) ||
      !std::isfinite(message.endpoint_map_m.x) ||
      !std::isfinite(message.endpoint_map_m.y) ||
      (message.endpoint_xyz_valid ? !std::isfinite(message.endpoint_map_m.z)
                                  : !std::isnan(message.endpoint_map_m.z)) ||
      !source_attitude_values_valid || !applied_attitude_values_valid ||
      (message.acquisition_stamp_valid && !acquisition_stamp.has_value()) ||
      (!message.acquisition_stamp_valid && !acquisition_stamp_zero) ||
      (message.receive_stamp_valid && !receive_stamp.has_value()) ||
      (!message.receive_stamp_valid && !receive_stamp_zero) ||
      (message.known_part_valid && !message.classifier_applied) ||
      (!message.classifier_applied &&
       (message.volume_matched || message.confident_face_interior ||
        !message.structure_id.empty() || !message.opening_id.empty() ||
        !message.part_id.empty())) ||
      (message.classifier_applied &&
       *classification == KnownStaticLidarHitClassification::kExpectedStatic)) {
    return std::nullopt;
  }

  AcceptedObstacleMemoryHit hit;
  hit.beam.beam_index = static_cast<std::size_t>(message.beam_index);
  if (message.acquisition_stamp_valid) {
    hit.beam.acquisition_stamp_ns = acquisition_stamp.value_or(0);
    hit.beam.acquisition_stamp_valid = true;
  }
  if (message.receive_stamp_valid) {
    hit.beam.receive_stamp_ns = receive_stamp.value_or(0);
    hit.beam.receive_stamp_valid = true;
  }
  hit.beam.projection.ray_origin_map_m =
      Point3{message.ray_origin_map_m.x, message.ray_origin_map_m.y,
             message.ray_origin_map_m.z};
  hit.beam.projection.ray_direction_map =
      Point3{message.ray_direction_map.x, message.ray_direction_map.y,
             message.ray_direction_map.z};
  hit.beam.projection.endpoint_map_m = Point3{
      message.endpoint_map_m.x, message.endpoint_map_m.y, message.endpoint_map_m.z};
  hit.beam.projection.endpoint =
      Point2{message.endpoint_map_m.x, message.endpoint_map_m.y};
  hit.beam.projection.endpoint_xyz_valid = message.endpoint_xyz_valid;
  hit.beam.measured_range_m = message.measured_range_m;
  hit.beam.source_attitude_valid = message.source_attitude_valid;
  hit.beam.source_roll_rad = message.source_roll_rad;
  hit.beam.source_pitch_rad = message.source_pitch_rad;
  hit.beam.source_tilt_rad = message.source_tilt_rad;
  hit.beam.projection.attitude_compensation_applied =
      message.attitude_compensation_applied;
  hit.beam.projection.applied_roll_rad = message.applied_roll_rad;
  hit.beam.projection.applied_pitch_rad = message.applied_pitch_rad;
  hit.beam.projection.applied_tilt_rad = message.applied_tilt_rad;
  hit.known_static.classifier_applied = message.classifier_applied;
  hit.known_static.classification = *classification;
  hit.known_static.volume_matched = message.volume_matched;
  hit.known_static.confident_face_interior = message.confident_face_interior;
  hit.known_static.part_kind_valid = message.known_part_valid;
  hit.known_static.part_kind = *part_kind;
  hit.known_static.structure_id = message.structure_id;
  hit.known_static.opening_id = message.opening_id;
  hit.known_static.part_id = message.part_id;
  hit.known_static.expected_range_m = message.expected_range_m;
  hit.known_static.range_delta_m = message.range_delta_m;
  hit.ingestion_decision = LidarIngestionDecisionSnapshot{
      .action = *ingestion_action,
      .reason = *ingestion_reason,
      .expected_surface = *ingestion_surface,
      .expected_range_m = message.ingestion_expected_range_m,
      .range_delta_m = message.ingestion_range_delta_m,
  };
  return hit;
}

[[nodiscard]] bool
endpointMatchesCell(const MemoryCellProvenance& record,
                    const nav_msgs::msg::MapMetaData& info) noexcept {
  const auto matches = [&info, &record](const AcceptedObstacleMemoryHit& hit) {
    const Point2 endpoint = hit.beam.projection.endpoint;
    const auto cell_x = finiteFloorToInt((endpoint.x - info.origin.position.x) /
                                         static_cast<double>(info.resolution));
    const auto cell_y = finiteFloorToInt((endpoint.y - info.origin.position.y) /
                                         static_cast<double>(info.resolution));
    if (!cell_x.has_value() || !cell_y.has_value()) {
      return false;
    }
    return *cell_x == record.cell.x && *cell_y == record.cell.y;
  };
  return matches(record.occupancy_trigger) && matches(record.last_hit);
}

[[nodiscard]] bool identitiesEqual(const MemoryGridSnapshotIdentity& lhs,
                                   const MemoryGridSnapshotIdentity& rhs) noexcept {
  return lhs.stamp_valid == rhs.stamp_valid && lhs.stamp_ns == rhs.stamp_ns &&
         lhs.frame_id == rhs.frame_id && sameGridInfo(lhs.grid_info, rhs.grid_info) &&
         lhs.raw_grid_data_hash == rhs.raw_grid_data_hash &&
         lhs.occupied_cell_count == rhs.occupied_cell_count;
}

[[nodiscard]] bool snapshotMatchesGrid(const MemoryProvenanceSnapshot& snapshot,
                                       const nav_msgs::msg::OccupancyGrid& grid) {
  if (snapshot.cells.size() != snapshot.identity.occupied_cell_count) {
    return false;
  }
  return std::ranges::all_of(snapshot.cells, [&grid](const auto& item) {
    const auto& [index, record] = item;
    return index < grid.data.size() && grid.data[index] == 100 &&
           endpointMatchesCell(record, grid.info);
  });
}

void appendHit(std::ostringstream& stream, const char* label,
               const AcceptedObstacleMemoryHit& hit) {
  stream << ' ' << label << "_endpoint=(" << hit.beam.projection.endpoint_map_m.x << ','
         << hit.beam.projection.endpoint_map_m.y << ','
         << hit.beam.projection.endpoint_map_m.z << ')' << ' ' << label << "_stamp=";
  if (hit.beam.acquisition_stamp_valid) {
    stream << hit.beam.acquisition_stamp_ns;
  } else {
    stream << "invalid";
  }
  stream << ' ' << label << "_attitude=(" << hit.beam.projection.applied_roll_rad << ','
         << hit.beam.projection.applied_pitch_rad << ','
         << hit.beam.projection.applied_tilt_rad << ')' << ' ' << label
         << "_range=" << hit.beam.measured_range_m << ' ' << label << "_classification="
         << knownStaticLidarHitClassificationName(hit.known_static.classification)
         << ' ' << label << "_part="
         << (hit.known_static.part_id.empty() ? "<none>" : hit.known_static.part_id)
         << ' ' << label << "_ingestion_action="
         << lidarIngestionActionName(hit.ingestion_decision.action) << ' ' << label
         << "_ingestion_reason="
         << lidarIngestionReasonName(hit.ingestion_decision.reason) << ' ' << label
         << "_ingestion_surface="
         << lidarExpectedSurfaceKindName(hit.ingestion_decision.expected_surface) << ' '
         << label
         << "_ingestion_expected_range=" << hit.ingestion_decision.expected_range_m
         << ' ' << label << "_ingestion_delta=" << hit.ingestion_decision.range_delta_m;
}

} // namespace

std::uint64_t rawGridDataHash(const std::span<const std::int8_t> data) noexcept {
  constexpr std::uint64_t kOffsetBasis = 14695981039346656037ULL;
  constexpr std::uint64_t kPrime = 1099511628211ULL;
  std::uint64_t hash = kOffsetBasis;
  for (const std::int8_t value : data) {
    hash ^= static_cast<std::uint8_t>(value);
    hash *= kPrime;
  }
  return hash;
}

MemoryGridSnapshotIdentity
memoryGridSnapshotIdentity(const nav_msgs::msg::OccupancyGrid& grid) {
  MemoryGridSnapshotIdentity identity;
  const auto stamp_ns = checkedStampNanoseconds(grid.header.stamp);
  identity.stamp_ns = stamp_ns.value_or(0);
  identity.stamp_valid = stamp_ns.has_value();
  identity.frame_id = grid.header.frame_id;
  identity.grid_info = grid.info;
  identity.raw_grid_data_hash = rawGridDataHash(grid.data);
  identity.occupied_cell_count = static_cast<std::uint64_t>(
      std::count(grid.data.begin(), grid.data.end(), static_cast<std::int8_t>(100)));
  return identity;
}

std::optional<MemoryGridSnapshotIdentity>
memoryProvenanceMessageIdentity(const msg::ObstacleMemoryProvenance& message) {
  const auto stamp_ns = checkedStampNanoseconds(message.header.stamp);
  if (!stamp_ns.has_value() || message.header.frame_id.empty() ||
      !validGridInfo(message.grid_info)) {
    return std::nullopt;
  }
  return MemoryGridSnapshotIdentity{
      .stamp_ns = *stamp_ns,
      .stamp_valid = true,
      .frame_id = message.header.frame_id,
      .grid_info = message.grid_info,
      .raw_grid_data_hash = message.raw_grid_data_hash,
      .occupied_cell_count = message.occupied_cell_count,
  };
}

msg::ObstacleMemoryProvenance makeObstacleMemoryProvenanceMessage(
    const nav_msgs::msg::OccupancyGrid& grid,
    const std::unordered_map<std::size_t, MemoryCellProvenance>& provenance) {
  msg::ObstacleMemoryProvenance message;
  message.header = grid.header;
  message.schema_version = msg::ObstacleMemoryProvenance::CURRENT_SCHEMA_VERSION;
  message.grid_info = grid.info;
  message.raw_grid_data_hash = rawGridDataHash(grid.data);
  message.occupied_cell_count = static_cast<std::uint64_t>(
      std::count(grid.data.begin(), grid.data.end(), static_cast<std::int8_t>(100)));

  std::vector<std::size_t> indices;
  indices.reserve(provenance.size());
  for (const auto& [index, record] : provenance) {
    (void)record;
    indices.push_back(index);
  }
  std::sort(indices.begin(), indices.end());
  message.cells.reserve(indices.size());
  for (const std::size_t index : indices) {
    const MemoryCellProvenance& record = provenance.at(index);
    msg::ObstacleMemoryCellProvenance cell;
    cell.cell_x = record.cell.x;
    cell.cell_y = record.cell.y;
    cell.occupancy_trigger = observationToMessage(record.occupancy_trigger);
    cell.last_hit = observationToMessage(record.last_hit);
    cell.endpoint_z_range_valid =
        record.min_endpoint_z_m.has_value() && record.max_endpoint_z_m.has_value();
    cell.min_endpoint_z_m =
        record.min_endpoint_z_m.value_or(std::numeric_limits<double>::quiet_NaN());
    cell.max_endpoint_z_m =
        record.max_endpoint_z_m.value_or(std::numeric_limits<double>::quiet_NaN());
    cell.accepted_hit_count = record.accepted_hit_count;
    message.cells.push_back(std::move(cell));
  }
  return message;
}

msg::ObstacleMemorySnapshot makeObstacleMemorySnapshotMessage(
    const nav_msgs::msg::OccupancyGrid& grid,
    const std::unordered_map<std::size_t, MemoryCellProvenance>& provenance,
    const std::uint64_t sequence, const std::uint64_t producer_instance_id) {
  msg::ObstacleMemorySnapshot message;
  message.producer_instance_id = producer_instance_id;
  message.sequence = sequence;
  message.grid = grid;
  message.provenance = makeObstacleMemoryProvenanceMessage(grid, provenance);
  return message;
}

std::size_t
serializedObstacleMemoryProvenanceSize(const msg::ObstacleMemoryProvenance& message) {
  rclcpp::Serialization<msg::ObstacleMemoryProvenance> serialization;
  rclcpp::SerializedMessage serialized_message;
  serialization.serialize_message(&message, &serialized_message);
  return serialized_message.get_rcl_serialized_message().buffer_length;
}

std::size_t
serializedObstacleMemorySnapshotSize(const msg::ObstacleMemorySnapshot& message) {
  rclcpp::Serialization<msg::ObstacleMemorySnapshot> serialization;
  rclcpp::SerializedMessage serialized_message;
  serialization.serialize_message(&message, &serialized_message);
  return serialized_message.get_rcl_serialized_message().buffer_length;
}

MemoryProvenanceParseResult
parseObstacleMemoryProvenanceMessage(const msg::ObstacleMemoryProvenance& message) {
  MemoryProvenanceParseResult result;
  if (message.schema_version != msg::ObstacleMemoryProvenance::CURRENT_SCHEMA_VERSION) {
    result.reason = MemoryProvenanceUnavailableReason::kSchemaInvalid;
    result.detail = "schema_version";
    return result;
  }
  if (!validGridInfo(message.grid_info)) {
    result.detail = "grid_info";
    return result;
  }
  if (message.header.frame_id.empty()) {
    result.detail = "frame_id";
    return result;
  }
  if (message.cells.size() != message.occupied_cell_count) {
    result.detail =
        "cell_count expected=" + std::to_string(message.occupied_cell_count) +
        " actual=" + std::to_string(message.cells.size());
    return result;
  }

  MemoryProvenanceSnapshot snapshot;
  const auto stamp_ns = checkedStampNanoseconds(message.header.stamp);
  snapshot.identity.stamp_ns = stamp_ns.value_or(0);
  snapshot.identity.stamp_valid = stamp_ns.has_value();
  snapshot.identity.frame_id = message.header.frame_id;
  snapshot.identity.grid_info = message.grid_info;
  snapshot.identity.raw_grid_data_hash = message.raw_grid_data_hash;
  snapshot.identity.occupied_cell_count = message.occupied_cell_count;
  if (!snapshot.identity.stamp_valid) {
    result.detail = "header_stamp";
    return result;
  }

  for (std::size_t message_index = 0U; message_index < message.cells.size();
       ++message_index) {
    const msg::ObstacleMemoryCellProvenance& cell = message.cells[message_index];
    if (cell.cell_x < 0 || cell.cell_y < 0 ||
        cell.cell_x >= static_cast<std::int32_t>(message.grid_info.width) ||
        cell.cell_y >= static_cast<std::int32_t>(message.grid_info.height)) {
      result.detail = "cell_bounds index=" + std::to_string(message_index);
      return result;
    }
    if (cell.accepted_hit_count == 0U) {
      result.detail = "accepted_hit_count index=" + std::to_string(message_index);
      return result;
    }
    if (cell.endpoint_z_range_valid &&
        (!std::isfinite(cell.min_endpoint_z_m) ||
         !std::isfinite(cell.max_endpoint_z_m) ||
         cell.min_endpoint_z_m > cell.max_endpoint_z_m)) {
      result.detail = "endpoint_z_range index=" + std::to_string(message_index);
      return result;
    }
    const auto trigger = observationFromMessage(cell.occupancy_trigger);
    const auto last = observationFromMessage(cell.last_hit);
    if (!trigger.has_value()) {
      result.detail = "occupancy_trigger index=" + std::to_string(message_index) +
                      " cell=(" + std::to_string(cell.cell_x) + "," +
                      std::to_string(cell.cell_y) + ')';
      return result;
    }
    if (!last.has_value()) {
      result.detail = "last_hit index=" + std::to_string(message_index) + " cell=(" +
                      std::to_string(cell.cell_x) + "," + std::to_string(cell.cell_y) +
                      ')';
      return result;
    }
    const std::size_t index = static_cast<std::size_t>(cell.cell_y) *
                                  static_cast<std::size_t>(message.grid_info.width) +
                              static_cast<std::size_t>(cell.cell_x);
    MemoryCellProvenance record;
    record.cell = GridIndex{cell.cell_x, cell.cell_y};
    record.occupancy_trigger = *trigger;
    record.last_hit = *last;
    if (cell.endpoint_z_range_valid) {
      record.min_endpoint_z_m = cell.min_endpoint_z_m;
      record.max_endpoint_z_m = cell.max_endpoint_z_m;
    }
    record.accepted_hit_count = cell.accepted_hit_count;
    if (!endpointMatchesCell(record, message.grid_info)) {
      result.detail = "endpoint_cell index=" + std::to_string(message_index);
      return result;
    }
    if (!snapshot.cells.emplace(index, std::move(record)).second) {
      result.detail = "duplicate_cell index=" + std::to_string(message_index);
      return result;
    }
  }

  result.snapshot = std::move(snapshot);
  result.reason = MemoryProvenanceUnavailableReason::kNone;
  result.detail = "ok";
  return result;
}

MemoryProvenanceParseResult
parseObstacleMemorySnapshotMessage(const msg::ObstacleMemorySnapshot& message) {
  MemoryProvenanceParseResult result =
      parseObstacleMemoryProvenanceMessage(message.provenance);
  if (!result.snapshot.has_value()) {
    return result;
  }

  const MemoryGridSnapshotIdentity grid_identity =
      memoryGridSnapshotIdentity(message.grid);
  const MemoryGridSnapshotIdentity& provenance_identity = result.snapshot->identity;
  if (!grid_identity.stamp_valid || !provenance_identity.stamp_valid ||
      grid_identity.stamp_ns != provenance_identity.stamp_ns) {
    result.snapshot.reset();
    result.reason = MemoryProvenanceUnavailableReason::kStampMismatch;
    result.detail = "snapshot_grid_stamp";
    return result;
  }
  if (grid_identity.frame_id != provenance_identity.frame_id) {
    result.snapshot.reset();
    result.reason = MemoryProvenanceUnavailableReason::kFrameMismatch;
    result.detail = "snapshot_grid_frame";
    return result;
  }
  if (!sameGridInfo(grid_identity.grid_info, provenance_identity.grid_info)) {
    result.snapshot.reset();
    result.reason = MemoryProvenanceUnavailableReason::kGeometryMismatch;
    result.detail = "snapshot_grid_geometry";
    return result;
  }
  if (grid_identity.raw_grid_data_hash != provenance_identity.raw_grid_data_hash ||
      grid_identity.occupied_cell_count != provenance_identity.occupied_cell_count ||
      !snapshotMatchesGrid(*result.snapshot, message.grid)) {
    result.snapshot.reset();
    result.reason = MemoryProvenanceUnavailableReason::kContentMismatch;
    result.detail = "snapshot_grid_content";
    return result;
  }
  return result;
}

MemoryProvenanceCache::MemoryProvenanceCache(const std::size_t capacity)
    : capacity_{std::max<std::size_t>(1U, capacity)} {
}

void MemoryProvenanceCache::insert(MemoryProvenanceSnapshot snapshot) {
  const auto existing = std::find_if(
      snapshots_.begin(), snapshots_.end(), [&snapshot](const auto& candidate) {
        return identitiesEqual(candidate.identity, snapshot.identity);
      });
  if (existing != snapshots_.end()) {
    *existing = std::move(snapshot);
    return;
  }
  snapshots_.push_back(std::move(snapshot));
  while (snapshots_.size() > capacity_) {
    snapshots_.pop_front();
  }
}

void MemoryProvenanceCache::clear() noexcept {
  snapshots_.clear();
}

std::size_t MemoryProvenanceCache::size() const noexcept {
  return snapshots_.size();
}

MemoryProvenanceMatchResult
MemoryProvenanceCache::match(const nav_msgs::msg::OccupancyGrid& grid) const {
  return match(grid, memoryGridSnapshotIdentity(grid));
}

MemoryProvenanceMatchResult
MemoryProvenanceCache::match(const nav_msgs::msg::OccupancyGrid& grid,
                             const MemoryGridSnapshotIdentity& grid_identity) const {
  if (snapshots_.empty()) {
    return {};
  }
  MemoryProvenanceUnavailableReason reason =
      MemoryProvenanceUnavailableReason::kStampMismatch;
  for (const MemoryProvenanceSnapshot& snapshot : snapshots_ | std::views::reverse) {
    const MemoryGridSnapshotIdentity& candidate = snapshot.identity;
    if (!candidate.stamp_valid || !grid_identity.stamp_valid ||
        candidate.stamp_ns != grid_identity.stamp_ns) {
      continue;
    }
    reason = MemoryProvenanceUnavailableReason::kFrameMismatch;
    if (candidate.frame_id != grid_identity.frame_id) {
      continue;
    }
    reason = MemoryProvenanceUnavailableReason::kGeometryMismatch;
    if (!sameGridInfo(candidate.grid_info, grid_identity.grid_info)) {
      continue;
    }
    reason = MemoryProvenanceUnavailableReason::kContentMismatch;
    if (candidate.raw_grid_data_hash != grid_identity.raw_grid_data_hash ||
        candidate.occupied_cell_count != grid_identity.occupied_cell_count ||
        !snapshotMatchesGrid(snapshot, grid)) {
      continue;
    }
    return MemoryProvenanceMatchResult{&snapshot,
                                       MemoryProvenanceUnavailableReason::kNone};
  }
  return MemoryProvenanceMatchResult{nullptr, reason};
}

const char* memoryProvenanceUnavailableReasonName(
    const MemoryProvenanceUnavailableReason reason) noexcept {
  switch (reason) {
    case MemoryProvenanceUnavailableReason::kNone:
      return "none";
    case MemoryProvenanceUnavailableReason::kNotApplicable:
      return "not_applicable";
    case MemoryProvenanceUnavailableReason::kNotReceived:
      return "not_received";
    case MemoryProvenanceUnavailableReason::kStampMismatch:
      return "stamp_mismatch";
    case MemoryProvenanceUnavailableReason::kFrameMismatch:
      return "frame_mismatch";
    case MemoryProvenanceUnavailableReason::kSchemaInvalid:
      return "schema_invalid";
    case MemoryProvenanceUnavailableReason::kGeometryMismatch:
      return "geometry_mismatch";
    case MemoryProvenanceUnavailableReason::kContentMismatch:
      return "content_mismatch";
    case MemoryProvenanceUnavailableReason::kCellMissing:
      return "cell_missing";
    case MemoryProvenanceUnavailableReason::kMalformed:
      return "malformed";
    case MemoryProvenanceUnavailableReason::kHistoryExpired:
      return "history_expired";
    case MemoryProvenanceUnavailableReason::kCapacityEvicted:
      return "capacity_evicted";
  }
  return "unknown";
}

std::string formatMemoryProvenanceDiagnostic(const MemoryProvenanceMatchResult& match,
                                             const std::optional<GridIndex> cell) {
  if (!cell.has_value()) {
    return "memory_provenance[status=not_applicable]";
  }
  if (match.snapshot == nullptr) {
    std::ostringstream stream;
    stream << "memory_provenance[status=unavailable reason="
           << memoryProvenanceUnavailableReasonName(match.reason) << ']';
    return stream.str();
  }
  const std::size_t index =
      static_cast<std::size_t>(cell->y) *
          static_cast<std::size_t>(match.snapshot->identity.grid_info.width) +
      static_cast<std::size_t>(cell->x);
  const auto record = match.snapshot->cells.find(index);
  if (record == match.snapshot->cells.end()) {
    return "memory_provenance[status=unavailable reason=cell_missing]";
  }

  std::ostringstream stream;
  stream << "memory_provenance[status=matched cell=(" << cell->x << ',' << cell->y
         << ')';
  appendHit(stream, "trigger", record->second.occupancy_trigger);
  appendHit(stream, "last", record->second.last_hit);
  stream << " z_range=";
  if (record->second.min_endpoint_z_m.has_value() &&
      record->second.max_endpoint_z_m.has_value()) {
    stream << '[' << record->second.min_endpoint_z_m.value_or(0.0) << ','
           << record->second.max_endpoint_z_m.value_or(0.0) << ']';
  } else {
    stream << "invalid";
  }
  stream << " accepted_hits=" << record->second.accepted_hit_count << ']';
  return stream.str();
}

} // namespace drone_city_nav
