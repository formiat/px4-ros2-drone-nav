#include "drone_city_nav/execution_horizon_contract_ros.hpp"

#include "drone_city_nav/committed_execution_authority_3d.hpp"
#include "drone_city_nav/control_contracts_3d.hpp"
#include "drone_city_nav/execution_horizon_timing.hpp"

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <ranges>
#include <string>

namespace drone_city_nav {
namespace {

static_assert(static_cast<std::uint8_t>(ExecutionAuthorityMode3D::kPlanned) ==
              msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED);
static_assert(static_cast<std::uint8_t>(ExecutionAuthorityMode3D::kPositionHold) ==
              msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD);
static_assert(static_cast<std::uint8_t>(ExecutionAuthorityMode3D::kRevoked) ==
              msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED);
static_assert(
    static_cast<std::uint8_t>(ExecutionAuthorityReason3D::kUnavailableWorld) ==
    msg::MppiTrajectoryHorizon::EXECUTION_REASON_UNAVAILABLE_WORLD);

constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};
constexpr std::uint64_t kAppliedControlContentDomain{0x4150504c4354524cULL};
constexpr std::uint64_t kAppliedControlWireDomain{0x4150504c57495245ULL};
constexpr std::uint64_t kExecutionHorizonContentDomain{0x45584543484f5249ULL};

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
    hash ^= (value >> shift) & 0xffU;
    hash *= kFnvPrime;
  }
}

[[nodiscard]] std::uint64_t canonicalFloatBits(float value) noexcept;
[[nodiscard]] std::uint64_t canonicalDoubleBits(double value) noexcept;

void hashString(std::uint64_t& hash, const std::string& value) noexcept {
  hashValue(hash, static_cast<std::uint64_t>(value.size()));
  for (const char character : value) {
    const auto byte = static_cast<unsigned char>(character);
    hash ^= byte;
    hash *= kFnvPrime;
  }
}

void hashPoint(std::uint64_t& hash, const geometry_msgs::msg::Point& point) noexcept {
  hashValue(hash, canonicalDoubleBits(point.x));
  hashValue(hash, canonicalDoubleBits(point.y));
  hashValue(hash, canonicalDoubleBits(point.z));
}

void hashVector(std::uint64_t& hash,
                const geometry_msgs::msg::Vector3& vector) noexcept {
  hashValue(hash, canonicalDoubleBits(vector.x));
  hashValue(hash, canonicalDoubleBits(vector.y));
  hashValue(hash, canonicalDoubleBits(vector.z));
}

[[nodiscard]] std::uint64_t canonicalFloatBits(const float value) noexcept {
  return value == 0.0F ? 0U : std::bit_cast<std::uint32_t>(value);
}

[[nodiscard]] std::uint64_t canonicalDoubleBits(const double value) noexcept {
  return value == 0.0 ? 0U : std::bit_cast<std::uint64_t>(value);
}

[[nodiscard]] bool floatRepresentable(const double value) noexcept {
  constexpr double kMaximumFloat =
      static_cast<double>(std::numeric_limits<float>::max());
  return std::isfinite(value) && value >= -kMaximumFloat && value <= kMaximumFloat;
}

[[nodiscard]] std::uint64_t
controlContentFingerprint(const msg::MppiControlFeedback& feedback,
                          const std::int64_t source_stamp_ns) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kAppliedControlContentDomain);
  hashValue(hash, feedback.producer_instance_id);
  hashValue(hash, feedback.horizon_producer_instance_id);
  hashValue(hash, feedback.horizon_sequence);
  hashValue(hash, static_cast<std::uint64_t>(source_stamp_ns));
  hashValue(hash, feedback.execution_mode);
  hashValue(hash, feedback.control_authoritative ? 1U : 0U);
  hashValue(hash, canonicalDoubleBits(feedback.acceleration.x));
  hashValue(hash, canonicalDoubleBits(feedback.acceleration.y));
  hashValue(hash, canonicalDoubleBits(feedback.acceleration.z));
  hashValue(hash, canonicalFloatBits(feedback.yaw_acceleration_radps2));
  hashValue(hash, canonicalFloatBits(feedback.yaw_rate_radps));
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] std::uint64_t
controlWireFingerprint(const msg::MppiControlFeedback& feedback) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kAppliedControlWireDomain);
  hashValue(hash, static_cast<std::uint64_t>(feedback.header.stamp.sec));
  hashValue(hash, feedback.header.stamp.nanosec);
  hashString(hash, feedback.header.frame_id);
  hashValue(hash, feedback.producer_instance_id);
  hashValue(hash, feedback.horizon_producer_instance_id);
  hashValue(hash, feedback.horizon_sequence);
  hashValue(hash, feedback.execution_mode);
  hashValue(hash, feedback.control_authoritative ? 1U : 0U);
  hashValue(hash, std::bit_cast<std::uint64_t>(feedback.acceleration.x));
  hashValue(hash, std::bit_cast<std::uint64_t>(feedback.acceleration.y));
  hashValue(hash, std::bit_cast<std::uint64_t>(feedback.acceleration.z));
  hashValue(hash, std::bit_cast<std::uint32_t>(feedback.yaw_acceleration_radps2));
  hashValue(hash, std::bit_cast<std::uint32_t>(feedback.yaw_rate_radps));
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] bool canonicalTime(const builtin_interfaces::msg::Time& time) noexcept {
  return time.nanosec < 1'000'000'000U;
}

[[nodiscard]] bool finitePoint(const geometry_msgs::msg::Point& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] bool
finiteControlFeedback(const msg::MppiControlFeedback& feedback) noexcept {
  return floatRepresentable(feedback.acceleration.x) &&
         floatRepresentable(feedback.acceleration.y) &&
         floatRepresentable(feedback.acceleration.z) &&
         std::isfinite(feedback.yaw_rate_radps) &&
         std::isfinite(feedback.yaw_acceleration_radps2);
}

[[nodiscard]] bool finitePoint(const msg::MppiHorizonPoint& point) noexcept {
  return point.time_from_start_ns >= 0 && std::isfinite(point.time_from_start_s) &&
         finitePoint(point.position) && std::isfinite(point.velocity.x) &&
         std::isfinite(point.velocity.y) && std::isfinite(point.velocity.z) &&
         std::isfinite(point.acceleration.x) && std::isfinite(point.acceleration.y) &&
         std::isfinite(point.acceleration.z) && std::isfinite(point.yaw_rad) &&
         std::isfinite(point.yaw_rate_radps) &&
         std::isfinite(point.yaw_acceleration_radps2);
}

[[nodiscard]] bool
validExactTiming(const msg::MppiTrajectoryHorizon& horizon) noexcept {
  if (!canonicalTime(horizon.header.stamp) || !canonicalTime(horizon.valid_from) ||
      !canonicalTime(horizon.valid_until)) {
    return false;
  }
  const std::optional<std::int64_t> terminal_offset_ns =
      executionHorizonTerminalOffsetNs(horizon.points.size(),
                                       horizon.control_interval_ns);
  if (!terminal_offset_ns) {
    return false;
  }
  for (std::size_t index = 0U; index < horizon.points.size(); ++index) {
    const std::int64_t expected_ns =
        static_cast<std::int64_t>(index) * horizon.control_interval_ns;
    const float expected_s =
        static_cast<float>(static_cast<double>(expected_ns) / 1'000'000'000.0);
    if (horizon.points[index].time_from_start_ns != expected_ns ||
        horizon.points[index].time_from_start_s != expected_s) {
      return false;
    }
  }
  const std::int64_t valid_from_ns =
      executionHorizonTimeNanoseconds(horizon.valid_from);
  const std::int64_t valid_until_ns =
      executionHorizonTimeNanoseconds(horizon.valid_until);
  return valid_from_ns > 0 && valid_until_ns > valid_from_ns &&
         *terminal_offset_ns <= valid_until_ns - valid_from_ns;
}

[[nodiscard]] bool
validRevocationTiming(const msg::MppiTrajectoryHorizon& horizon) noexcept {
  if (!canonicalTime(horizon.header.stamp) || !canonicalTime(horizon.valid_from) ||
      !canonicalTime(horizon.valid_until)) {
    return false;
  }
  const std::int64_t valid_from_ns =
      executionHorizonTimeNanoseconds(horizon.valid_from);
  return valid_from_ns > 0 &&
         executionHorizonTimeNanoseconds(horizon.valid_until) == valid_from_ns &&
         horizon.control_interval_ns == 0 && horizon.points.empty();
}

[[nodiscard]] bool failClosedReason(const std::uint8_t reason) noexcept {
  return reason == msg::MppiTrajectoryHorizon::EXECUTION_REASON_NO_EXECUTABLE_HORIZON ||
         reason == msg::MppiTrajectoryHorizon::EXECUTION_REASON_NO_EXECUTABLE_ROUTE ||
         reason == msg::MppiTrajectoryHorizon::EXECUTION_REASON_UNAVAILABLE_WORLD;
}

[[nodiscard]] bool zeroPoint(const geometry_msgs::msg::Point& point) noexcept {
  return point.x == 0.0 && point.y == 0.0 && point.z == 0.0;
}

[[nodiscard]] bool zeroMotion(const msg::MppiHorizonPoint& point) noexcept {
  return point.velocity.x == 0.0 && point.velocity.y == 0.0 &&
         point.velocity.z == 0.0 && point.acceleration.x == 0.0 &&
         point.acceleration.y == 0.0 && point.acceleration.z == 0.0 &&
         point.yaw_rate_radps == 0.0F && point.yaw_acceleration_radps2 == 0.0F;
}

[[nodiscard]] bool terminalRest(const msg::MppiHorizonPoint& point) noexcept {
  // The same two tolerances the horizon builder shapes its arrival to and the
  // certificate admits it under.
  constexpr double kVelocityTolerance{kTerminalRestVelocityToleranceMps};
  constexpr double kControlTolerance{kTerminalRestControlToleranceMps2};
  return std::hypot(std::hypot(point.velocity.x, point.velocity.y), point.velocity.z) <=
             kVelocityTolerance &&
         std::abs(point.yaw_rate_radps) <= kVelocityTolerance &&
         std::abs(point.acceleration.x) <= kControlTolerance &&
         std::abs(point.acceleration.y) <= kControlTolerance &&
         std::abs(point.acceleration.z) <= kControlTolerance &&
         std::abs(point.yaw_acceleration_radps2) <= kControlTolerance;
}

[[nodiscard]] bool samePosition(const geometry_msgs::msg::Point& first,
                                const geometry_msgs::msg::Point& second) noexcept {
  return first.x == second.x && first.y == second.y && first.z == second.z;
}

} // namespace

std::int64_t
executionHorizonTimeNanoseconds(const builtin_interfaces::msg::Time& time) noexcept {
  return static_cast<std::int64_t>(time.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(time.nanosec);
}

std::uint64_t
executionHorizonContentFingerprint(const msg::MppiTrajectoryHorizon& horizon) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kExecutionHorizonContentDomain);
  hashValue(hash, static_cast<std::uint64_t>(horizon.header.stamp.sec));
  hashValue(hash, horizon.header.stamp.nanosec);
  hashString(hash, horizon.header.frame_id);
  hashValue(hash, horizon.sequence);
  hashValue(hash, static_cast<std::uint64_t>(horizon.valid_from.sec));
  hashValue(hash, horizon.valid_from.nanosec);
  hashValue(hash, static_cast<std::uint64_t>(horizon.valid_until.sec));
  hashValue(hash, horizon.valid_until.nanosec);
  hashValue(hash, static_cast<std::uint64_t>(horizon.control_interval_ns));
  hashValue(hash, horizon.pose_revision);
  hashValue(hash, horizon.obstacle_revision);
  hashValue(hash, horizon.risk_tier);
  hashValue(hash, horizon.execution_mode);
  hashValue(hash, horizon.execution_reason);
  hashPoint(hash, horizon.route_target);
  hashValue(hash, horizon.route_constrained ? 1U : 0U);
  hashValue(hash, horizon.stationary_position_hold ? 1U : 0U);
  hashPoint(hash, horizon.stationary_hold_position);
  hashValue(hash, horizon.producer_instance_id);
  hashValue(hash, horizon.target_offboard_instance_id);
  hashValue(hash, static_cast<std::uint64_t>(horizon.points.size()));
  for (const msg::MppiHorizonPoint& point : horizon.points) {
    hashValue(hash, canonicalFloatBits(point.time_from_start_s));
    hashValue(hash, static_cast<std::uint64_t>(point.time_from_start_ns));
    hashPoint(hash, point.position);
    hashVector(hash, point.velocity);
    hashVector(hash, point.acceleration);
    hashValue(hash, canonicalFloatBits(point.yaw_rad));
    hashValue(hash, canonicalFloatBits(point.yaw_rate_radps));
    hashValue(hash, canonicalFloatBits(point.yaw_acceleration_radps2));
  }
  return hash == 0U ? 1U : hash;
}

ExecutionHorizonPayloadStatus assessExecutionHorizonPayload(
    const msg::MppiTrajectoryHorizon& horizon,
    const ExecutionHorizonPayloadValidationConfig& config) noexcept {
  if (horizon.producer_instance_id == 0U || horizon.sequence == 0U ||
      horizon.target_offboard_instance_id == 0U ||
      executionHorizonTimeNanoseconds(horizon.header.stamp) <= 0) {
    return ExecutionHorizonPayloadStatus::kInvalidIdentity;
  }
  if (config.expected_frame_id.empty() ||
      horizon.header.frame_id != config.expected_frame_id) {
    return ExecutionHorizonPayloadStatus::kInvalidFrame;
  }
  if (horizon.execution_mode > msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED ||
      horizon.execution_reason >
          msg::MppiTrajectoryHorizon::EXECUTION_REASON_UNAVAILABLE_WORLD) {
    return ExecutionHorizonPayloadStatus::kInvalidEnum;
  }
  const bool revoked =
      horizon.execution_mode == msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED;
  if (!(revoked ? validRevocationTiming(horizon) : validExactTiming(horizon))) {
    return ExecutionHorizonPayloadStatus::kInvalidTiming;
  }
  if (revoked) {
    if (!failClosedReason(horizon.execution_reason) ||
        horizon.stationary_position_hold || horizon.route_constrained ||
        horizon.pose_revision != 0U || horizon.obstacle_revision != 0U ||
        horizon.risk_tier != 0U || !zeroPoint(horizon.route_target) ||
        !zeroPoint(horizon.stationary_hold_position)) {
      return ExecutionHorizonPayloadStatus::kInconsistentRevocation;
    }
    return ExecutionHorizonPayloadStatus::kValid;
  }
  const bool position_hold_mode =
      horizon.execution_mode ==
      msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD;
  if (horizon.stationary_position_hold != position_hold_mode) {
    return ExecutionHorizonPayloadStatus::kInconsistentExecutionMode;
  }
  if (!finitePoint(horizon.route_target) ||
      (position_hold_mode && !finitePoint(horizon.stationary_hold_position))) {
    return ExecutionHorizonPayloadStatus::kNonFiniteMetadata;
  }
  if (!std::ranges::all_of(horizon.points, [](const msg::MppiHorizonPoint& point) {
        return finitePoint(point);
      })) {
    return ExecutionHorizonPayloadStatus::kNonFinitePoint;
  }
  if (config.flight_envelope != nullptr &&
      (!insideFlightEnvelope(horizon.route_target.z, *config.flight_envelope) ||
       !std::ranges::all_of(horizon.points,
                            [&config](const msg::MppiHorizonPoint& point) {
                              return insideFlightEnvelope(point.position.z,
                                                          *config.flight_envelope);
                            }) ||
       (position_hold_mode && !insideFlightEnvelope(horizon.stationary_hold_position.z,
                                                    *config.flight_envelope)))) {
    return ExecutionHorizonPayloadStatus::kOutsideFlightEnvelope;
  }
  if (!position_hold_mode && !terminalRest(horizon.points.back())) {
    return ExecutionHorizonPayloadStatus::kMissingTerminalRestState;
  }
  if (position_hold_mode &&
      !std::ranges::all_of(
          horizon.points, [&horizon](const msg::MppiHorizonPoint& point) {
            return samePosition(point.position, horizon.stationary_hold_position) &&
                   zeroMotion(point);
          })) {
    return ExecutionHorizonPayloadStatus::kInconsistentStationaryHold;
  }
  return ExecutionHorizonPayloadStatus::kValid;
}

std::string_view
executionHorizonPayloadStatusName(const ExecutionHorizonPayloadStatus status) noexcept {
  switch (status) {
    case ExecutionHorizonPayloadStatus::kValid:
      return "valid";
    case ExecutionHorizonPayloadStatus::kInvalidIdentity:
      return "invalid_identity";
    case ExecutionHorizonPayloadStatus::kInvalidFrame:
      return "invalid_frame";
    case ExecutionHorizonPayloadStatus::kInvalidTiming:
      return "invalid_timing";
    case ExecutionHorizonPayloadStatus::kNonFiniteMetadata:
      return "non_finite_metadata";
    case ExecutionHorizonPayloadStatus::kNonFinitePoint:
      return "non_finite_point";
    case ExecutionHorizonPayloadStatus::kOutsideFlightEnvelope:
      return "outside_flight_envelope";
    case ExecutionHorizonPayloadStatus::kInvalidEnum:
      return "invalid_enum";
    case ExecutionHorizonPayloadStatus::kInconsistentExecutionMode:
      return "inconsistent_execution_mode";
    case ExecutionHorizonPayloadStatus::kInconsistentRevocation:
      return "inconsistent_revocation";
    case ExecutionHorizonPayloadStatus::kMissingTerminalRestState:
      return "missing_terminal_rest_state";
    case ExecutionHorizonPayloadStatus::kInconsistentStationaryHold:
      return "inconsistent_stationary_hold";
  }
  return "unknown";
}

ExecutionControlFeedbackAssessment
assessExecutionControlFeedback(const msg::MppiControlFeedback& feedback,
                               const std::string_view expected_frame_id,
                               const std::int64_t receive_stamp_ns) noexcept {
  const std::int64_t source_stamp_ns =
      executionHorizonTimeNanoseconds(feedback.header.stamp);
  const bool heartbeat_identity =
      feedback.horizon_producer_instance_id == 0U && feedback.horizon_sequence == 0U;
  const bool horizon_identity =
      feedback.horizon_producer_instance_id != 0U && feedback.horizon_sequence != 0U;
  ExecutionControlFeedbackAssessment assessment{
      .candidate =
          ExecutionHorizonFeedbackCandidate{
              .offboard_producer_instance_id = feedback.producer_instance_id,
              .horizon_producer_instance_id = feedback.horizon_producer_instance_id,
              .horizon_sequence = feedback.horizon_sequence,
              .source_stamp_ns = source_stamp_ns,
              .receive_stamp_ns = receive_stamp_ns,
              .content_fingerprint = horizon_identity ? controlContentFingerprint(
                                                            feedback, source_stamp_ns)
                                                      : 0U,
              .wire_fingerprint = controlWireFingerprint(feedback),
              .execution_mode =
                  static_cast<ExecutionHorizonWitnessMode>(feedback.execution_mode),
              .control_authoritative = feedback.control_authoritative,
          },
  };
  if (expected_frame_id.empty() || feedback.header.frame_id != expected_frame_id) {
    assessment.status = ExecutionControlFeedbackStatus::kInvalidFrame;
    return assessment;
  }
  if (!canonicalTime(feedback.header.stamp)) {
    assessment.status = ExecutionControlFeedbackStatus::kInvalidTiming;
    return assessment;
  }

  // Source time identifies the offboard control sample while receipt time is
  // local evidence of delivery.  ROS /clock is distributed asynchronously in
  // simulation, so their ordering is not an authority invariant.
  if (source_stamp_ns <= 0 || receive_stamp_ns <= 0) {
    assessment.status = ExecutionControlFeedbackStatus::kInvalidTiming;
    return assessment;
  }
  if (feedback.producer_instance_id == 0U) {
    assessment.status = ExecutionControlFeedbackStatus::kInvalidIdentity;
    return assessment;
  }
  if (!heartbeat_identity && !horizon_identity) {
    assessment.status = ExecutionControlFeedbackStatus::kInvalidIdentity;
    return assessment;
  }
  if (feedback.execution_mode >
      msg::MppiControlFeedback::EXECUTION_MODE_POSITION_HOLD) {
    assessment.status = ExecutionControlFeedbackStatus::kInvalidEnum;
    return assessment;
  }
  const bool planned =
      feedback.execution_mode == msg::MppiControlFeedback::EXECUTION_MODE_PLANNED;
  if ((heartbeat_identity && (planned || feedback.control_authoritative)) ||
      (horizon_identity && feedback.control_authoritative && !planned)) {
    assessment.status = ExecutionControlFeedbackStatus::kInconsistentExecutionMode;
    return assessment;
  }
  if (!finiteControlFeedback(feedback)) {
    assessment.status = ExecutionControlFeedbackStatus::kNonFiniteControl;
    return assessment;
  }
  assessment.status = assessment.candidate.valid()
                          ? ExecutionControlFeedbackStatus::kValid
                          : ExecutionControlFeedbackStatus::kInvalidIdentity;
  return assessment;
}

std::string_view executionControlFeedbackStatusName(
    const ExecutionControlFeedbackStatus status) noexcept {
  switch (status) {
    case ExecutionControlFeedbackStatus::kValid:
      return "valid";
    case ExecutionControlFeedbackStatus::kInvalidFrame:
      return "invalid_frame";
    case ExecutionControlFeedbackStatus::kInvalidTiming:
      return "invalid_timing";
    case ExecutionControlFeedbackStatus::kInvalidIdentity:
      return "invalid_identity";
    case ExecutionControlFeedbackStatus::kInvalidEnum:
      return "invalid_enum";
    case ExecutionControlFeedbackStatus::kInconsistentExecutionMode:
      return "inconsistent_execution_mode";
    case ExecutionControlFeedbackStatus::kNonFiniteControl:
      return "non_finite_control";
  }
  return "unknown";
}

} // namespace drone_city_nav
