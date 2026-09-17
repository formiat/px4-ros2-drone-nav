#include "drone_city_nav/px4_autopilot_adapter.hpp"

#include "drone_city_nav/autopilot_state_source.hpp"

#include <bit>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kPayloadFingerprintOffset{14'695'981'039'346'656'037ULL};
constexpr std::uint64_t kPayloadFingerprintPrime{1'099'511'628'211ULL};

void appendFingerprint(std::uint64_t& fingerprint, const std::uint64_t word) noexcept {
  for (std::size_t byte_index = 0U; byte_index < sizeof(word); ++byte_index) {
    fingerprint ^= (word >> (byte_index * 8U)) & 0xFFU;
    fingerprint *= kPayloadFingerprintPrime;
  }
}

[[nodiscard]] std::uint32_t canonicalFloatBits(const float value) noexcept {
  static_assert(sizeof(float) == sizeof(std::uint32_t));
  if (value == 0.0F) {
    return 0U;
  }
  if (std::isnan(value)) {
    return 0x7FC0'0000U;
  }
  return std::bit_cast<std::uint32_t>(value);
}

// Every field of the sample the estimator can change without touching its
// stamps, so a repeated stamp with new content is a new sample.
[[nodiscard]] std::uint64_t
payloadFingerprint(const px4_msgs::msg::VehicleLocalPosition& message) noexcept {
  std::uint64_t fingerprint = kPayloadFingerprintOffset;
  const auto append_float = [&](const float value) noexcept {
    appendFingerprint(fingerprint, canonicalFloatBits(value));
  };
  appendFingerprint(fingerprint, 1U); // Fingerprint schema.
  append_float(message.x);
  append_float(message.y);
  append_float(message.z);
  append_float(message.delta_xy[0]);
  append_float(message.delta_xy[1]);
  append_float(message.delta_z);
  append_float(message.vx);
  append_float(message.vy);
  append_float(message.vz);
  append_float(message.delta_vxy[0]);
  append_float(message.delta_vxy[1]);
  append_float(message.delta_vz);
  append_float(message.ax);
  append_float(message.ay);
  append_float(message.az);
  append_float(message.heading);
  append_float(message.delta_heading);
  appendFingerprint(fingerprint, message.xy_reset_counter);
  appendFingerprint(fingerprint, message.z_reset_counter);
  appendFingerprint(fingerprint, message.vxy_reset_counter);
  appendFingerprint(fingerprint, message.vz_reset_counter);
  appendFingerprint(fingerprint, message.heading_reset_counter);
  appendFingerprint(fingerprint, message.xy_valid ? 1U : 0U);
  appendFingerprint(fingerprint, message.z_valid ? 1U : 0U);
  appendFingerprint(fingerprint, message.v_xy_valid ? 1U : 0U);
  appendFingerprint(fingerprint, message.v_z_valid ? 1U : 0U);
  appendFingerprint(fingerprint, message.heading_good_for_control ? 1U : 0U);
  return fingerprint;
}

} // namespace

AutopilotLocalState
px4LocalPositionToAutopilotState(const px4_msgs::msg::VehicleLocalPosition& message,
                                 const Px4MapFrameTransform& transform) noexcept {
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  AutopilotLocalState state;
  state.timestamp_us = message.timestamp;
  state.timestamp_sample_us = message.timestamp_sample;
  const Point2 position = transform.localPositionToMap(
      Point2{static_cast<double>(message.x), static_cast<double>(message.y)});
  state.position = Point3{position.x, position.y,
                          -static_cast<double>(message.z) + transform.map_origin.z};
  const Point2 velocity = transform.localVectorToMap(
      Point2{static_cast<double>(message.vx), static_cast<double>(message.vy)});
  state.velocity = Vec3{velocity.x, velocity.y, -static_cast<double>(message.vz)};
  if (std::isfinite(message.ax) && std::isfinite(message.ay) &&
      std::isfinite(message.az)) {
    const Point2 acceleration = transform.localVectorToMap(
        Point2{static_cast<double>(message.ax), static_cast<double>(message.ay)});
    state.acceleration =
        Vec3{acceleration.x, acceleration.y, -static_cast<double>(message.az)};
  } else {
    state.acceleration = Vec3{nan, nan, nan};
  }
  state.yaw_rad = std::isfinite(message.heading)
                      ? transform.px4HeadingToMapYaw(message.heading)
                      : nan;
  state.heading_variance_rad2 = static_cast<double>(message.heading_var);
  state.position_valid = message.xy_valid;
  state.altitude_valid = message.z_valid;
  state.velocity_valid = message.v_xy_valid;
  state.vertical_velocity_valid = message.v_z_valid;
  state.heading_valid =
      message.heading_good_for_control && std::isfinite(message.heading);
  state.xy_reset_counter = message.xy_reset_counter;
  state.z_reset_counter = message.z_reset_counter;
  state.vxy_reset_counter = message.vxy_reset_counter;
  state.vz_reset_counter = message.vz_reset_counter;
  state.heading_reset_counter = message.heading_reset_counter;
  state.payload_fingerprint = payloadFingerprint(message);
  return state;
}

AutopilotAttitude
px4AttitudeToAutopilotAttitude(const px4_msgs::msg::VehicleAttitude& message) noexcept {
  return AutopilotAttitude{.timestamp_us = message.timestamp,
                           .timestamp_sample_us = message.timestamp_sample,
                           .quaternion = message.q};
}

AutopilotImuSample
px4SensorCombinedToAutopilotImu(const px4_msgs::msg::SensorCombined& message) noexcept {
  return AutopilotImuSample{
      .timestamp_us = message.timestamp,
      .gyro_radps = Vec3{static_cast<double>(message.gyro_rad[0]),
                         static_cast<double>(message.gyro_rad[1]),
                         static_cast<double>(message.gyro_rad[2])},
      .accelerometer_mps2 = Vec3{static_cast<double>(message.accelerometer_m_s2[0]),
                                 static_cast<double>(message.accelerometer_m_s2[1]),
                                 static_cast<double>(message.accelerometer_m_s2[2])},
  };
}

px4_msgs::msg::VehicleOdometry
px4VisualOdometryFromEstimate(const LidarInertialEstimate& estimate,
                              const std::uint64_t timestamp_sample_us) noexcept {
  px4_msgs::msg::VehicleOdometry odometry;
  odometry.timestamp = 0U;
  odometry.timestamp_sample = timestamp_sample_us;
  odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  odometry.position = {static_cast<float>(estimate.position_ned_m.x()),
                       static_cast<float>(estimate.position_ned_m.y()),
                       static_cast<float>(estimate.position_ned_m.z())};
  odometry.q = {static_cast<float>(estimate.body_to_ned.w()),
                static_cast<float>(estimate.body_to_ned.x()),
                static_cast<float>(estimate.body_to_ned.y()),
                static_cast<float>(estimate.body_to_ned.z())};
  odometry.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
  odometry.velocity = {static_cast<float>(estimate.velocity_ned_mps.x()),
                       static_cast<float>(estimate.velocity_ned_mps.y()),
                       static_cast<float>(estimate.velocity_ned_mps.z())};
  constexpr float nan = std::numeric_limits<float>::quiet_NaN();
  odometry.angular_velocity = {nan, nan, nan};
  odometry.position_variance = {static_cast<float>(estimate.position_variance_m2.x()),
                                static_cast<float>(estimate.position_variance_m2.y()),
                                static_cast<float>(estimate.position_variance_m2.z())};
  odometry.orientation_variance = {
      static_cast<float>(estimate.orientation_variance_rad2.x()),
      static_cast<float>(estimate.orientation_variance_rad2.y()),
      static_cast<float>(estimate.orientation_variance_rad2.z())};
  odometry.velocity_variance = {
      static_cast<float>(estimate.velocity_variance_m2ps2.x()),
      static_cast<float>(estimate.velocity_variance_m2ps2.y()),
      static_cast<float>(estimate.velocity_variance_m2ps2.z())};
  odometry.reset_counter = 0U;
  odometry.quality = estimate.healthy ? 100 : 0;
  return odometry;
}

AutopilotClockSync
px4TimesyncToAutopilotClockSync(const px4_msgs::msg::TimesyncStatus& message) noexcept {
  return AutopilotClockSync{.timestamp_us = message.timestamp,
                            .estimated_offset_us = message.estimated_offset,
                            .round_trip_time_us = message.round_trip_time};
}

AutopilotStatus px4VehicleStatusToAutopilotStatus(
    const px4_msgs::msg::VehicleStatus& message) noexcept {
  return AutopilotStatus{
      .timestamp_us = message.timestamp,
      .armed = message.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED,
      .external_control =
          message.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD,
  };
}

AutopilotGroundContact px4LandDetectedToAutopilotGroundContact(
    const px4_msgs::msg::VehicleLandDetected& message) noexcept {
  return AutopilotGroundContact{.detected = message.landed || message.maybe_landed ||
                                            message.ground_contact};
}

AutopilotStateSource::AutopilotStateSource(
    rclcpp::Node& node, const Px4MapFrameTransform& transform,
    const AutopilotStateTopics& topics, const rclcpp::QoS& qos,
    AutopilotStateCallbacks callbacks, const rclcpp::SubscriptionOptions& state_options,
    const rclcpp::SubscriptionOptions& status_options) {
  if (!topics.local_state.empty() && callbacks.local_state) {
    subscriptions_.push_back(
        node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            topics.local_state, qos,
            [transform, callback = std::move(callbacks.local_state)](
                const px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
              callback(px4LocalPositionToAutopilotState(*message, transform));
            },
            state_options));
  }
  if (!topics.attitude.empty() && callbacks.attitude) {
    subscriptions_.push_back(node.create_subscription<px4_msgs::msg::VehicleAttitude>(
        topics.attitude, qos,
        [callback = std::move(callbacks.attitude)](
            const px4_msgs::msg::VehicleAttitude::SharedPtr message) {
          callback(px4AttitudeToAutopilotAttitude(*message));
        },
        state_options));
  }
  if (!topics.imu.empty() && callbacks.imu) {
    subscriptions_.push_back(node.create_subscription<px4_msgs::msg::SensorCombined>(
        topics.imu, qos,
        [callback = std::move(callbacks.imu)](
            const px4_msgs::msg::SensorCombined::SharedPtr message) {
          callback(px4SensorCombinedToAutopilotImu(*message));
        },
        state_options));
  }
  if (!topics.clock_sync.empty() && callbacks.clock_sync) {
    subscriptions_.push_back(node.create_subscription<px4_msgs::msg::TimesyncStatus>(
        topics.clock_sync, qos,
        [callback = std::move(callbacks.clock_sync)](
            const px4_msgs::msg::TimesyncStatus::SharedPtr message) {
          callback(px4TimesyncToAutopilotClockSync(*message));
        },
        state_options));
  }
  if (!topics.status.empty() && callbacks.status) {
    subscriptions_.push_back(node.create_subscription<px4_msgs::msg::VehicleStatus>(
        topics.status, qos,
        [callback = std::move(callbacks.status)](
            const px4_msgs::msg::VehicleStatus::SharedPtr message) {
          callback(px4VehicleStatusToAutopilotStatus(*message));
        },
        status_options));
  }
  if (!topics.ground_contact.empty() && callbacks.ground_contact) {
    subscriptions_.push_back(
        node.create_subscription<px4_msgs::msg::VehicleLandDetected>(
            topics.ground_contact, qos,
            [callback = std::move(callbacks.ground_contact)](
                const px4_msgs::msg::VehicleLandDetected::SharedPtr message) {
              callback(px4LandDetectedToAutopilotGroundContact(*message));
            },
            status_options));
  }
}

} // namespace drone_city_nav
