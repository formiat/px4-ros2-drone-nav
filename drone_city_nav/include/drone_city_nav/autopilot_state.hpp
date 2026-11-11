#pragma once

#include "drone_city_nav/types.hpp"

#include <array>
#include <cmath>
#include <cstdint>

namespace drone_city_nav {

// The state of the autopilot as the navigation stack reads it. One adapter
// per autopilot fills these from the autopilot's own messages; the stack reads
// nothing else. Positions, velocities, accelerations and yaw are in the map
// frame (z up) with the map origin applied: the autopilot's own frame, the
// sign of its altitude and its heading convention are the adapter's business.
// Timestamps stay in the autopilot's clock, in microseconds; the stack maps
// them to ROS time itself, from the clock synchronisation samples.
struct AutopilotLocalState {
  std::uint64_t timestamp_us{0U};
  std::uint64_t timestamp_sample_us{0U};
  Point3 position{};
  Vec3 velocity{};
  // NaN where the autopilot reports none.
  Vec3 acceleration{};
  // Map yaw; meaningful only while heading_valid.
  double yaw_rad{0.0};
  double heading_variance_rad2{0.0};
  bool position_valid{false};
  bool altitude_valid{false};
  bool velocity_valid{false};
  bool vertical_velocity_valid{false};
  // The autopilot vouches for the heading as good enough to control on.
  bool heading_valid{false};
  // The autopilot's own reset counters: a change means the estimate jumped
  // and the continuity of the frame has to be re-established.
  std::uint8_t xy_reset_counter{0U};
  std::uint8_t z_reset_counter{0U};
  std::uint8_t vxy_reset_counter{0U};
  std::uint8_t vz_reset_counter{0U};
  std::uint8_t heading_reset_counter{0U};
  // A fingerprint of the autopilot's own sample payload, so two samples with
  // the same stamps are told apart by content.
  std::uint64_t payload_fingerprint{0U};
};

struct AutopilotAttitude {
  std::uint64_t timestamp_us{0U};
  std::uint64_t timestamp_sample_us{0U};
  // The body attitude as a Hamilton quaternion (w, x, y, z): the rotation of
  // the world NED frame into the body FRD frame's orientation, which turns a
  // body vector into NED (PX4's vehicle_attitude.q). The lidar registration and the
  // tilt laws read this convention; an adapter for an autopilot with another
  // convention converts into it.
  std::array<float, 4> quaternion{1.0F, 0.0F, 0.0F, 0.0F};
};

// One sample of the autopilot's IMU in the body FRD frame, in the
// autopilot's clock: the lidar-inertial estimator's inertial input.
struct AutopilotImuSample {
  std::uint64_t timestamp_us{0U};
  Vec3 gyro_radps{};
  Vec3 accelerometer_mps2{};
};

// One fix of the autopilot's satellite receiver (roadmap item 39): what the
// visual-inertial estimator reads on the hybrid profile. The receiver's own
// accuracy figures ride with it; a fix without a 3D solution is no fix.
struct AutopilotGnssFix {
  std::uint64_t timestamp_us{0U};
  bool fix_3d{false};
  std::uint8_t fix_type{0U};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_msl_m{0.0};
  Vec3 velocity_ned_mps{};
  double horizontal_accuracy_m{0.0}; // eph
  double vertical_accuracy_m{0.0};   // epv
  double speed_accuracy_mps{0.0};
};

// One sample of the autopilot clock against the stack's clock.
struct AutopilotClockSync {
  std::uint64_t timestamp_us{0U};
  std::int64_t estimated_offset_us{0};
  std::uint32_t round_trip_time_us{0U};
};

struct AutopilotStatus {
  std::uint64_t timestamp_us{0U};
  bool armed{false};
  // The autopilot flies the stack's setpoints (PX4: offboard mode).
  bool external_control{false};
  // The autopilot lands the vehicle by a mode of its own (PX4: Land, Descend).
  bool autopilot_landing{false};
  // The autopilot acts on a failsafe of its own.
  bool failsafe{false};
  // The flight was terminated or the motors killed: by a kill switch, a
  // lockdown, a forced disarm or the autopilot's failure detector.
  bool terminated{false};
};

// The links the autopilot reports (specification K29): the pilot's radio and
// the ground station. Lost means the autopilot has stopped hearing one.
struct AutopilotLinks {
  std::uint64_t timestamp_us{0U};
  bool manual_control_lost{false};
  bool ground_station_lost{false};
};

// The vehicle's main battery (specification K32): the share of its charge
// left, and the time left the autopilot estimates, NaN where it has none.
struct AutopilotBattery {
  std::uint64_t timestamp_us{0U};
  double remaining{0.0};
  double time_remaining_s{0.0};
};

// The autopilot's own landing detector: any contact with the ground.
struct AutopilotGroundContact {
  bool detected{false};
};

// A reset of the autopilot's estimate (its xy or z reset counter changed) is
// a new frame when it moved the estimate farther than the tolerance, the new
// position read against the previous one carried at its velocity over the
// receive interval, or when the interval is too long to carry it (specification
// K77). Within the tolerance it is the same frame: the autopilot aligning its
// estimator moves its position by centimetres (r372).
inline constexpr double kAutopilotFrameResetToleranceM{3.0};
inline constexpr double kAutopilotFrameResetMaximumIntervalS{0.2};

[[nodiscard]] inline bool autopilotFrameReset(const AutopilotLocalState& previous,
                                              const AutopilotLocalState& current,
                                              const double interval_s) noexcept {
  if (current.xy_reset_counter == previous.xy_reset_counter &&
      current.z_reset_counter == previous.z_reset_counter) {
    return false;
  }
  if (!(interval_s > 0.0) || interval_s > kAutopilotFrameResetMaximumIntervalS) {
    return true;
  }
  const double dx =
      current.position.x - (previous.position.x + previous.velocity.x * interval_s);
  const double dy =
      current.position.y - (previous.position.y + previous.velocity.y * interval_s);
  const double dz =
      current.position.z - (previous.position.z + previous.velocity.z * interval_s);
  return !(std::sqrt(dx * dx + dy * dy + dz * dz) <= kAutopilotFrameResetToleranceM);
}

} // namespace drone_city_nav
