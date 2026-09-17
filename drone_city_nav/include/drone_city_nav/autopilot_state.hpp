#pragma once

#include "drone_city_nav/types.hpp"

#include <array>
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
  // The body attitude as a Hamilton quaternion (w, x, y, z) rotating the
  // world NED frame into the body FRD frame. The lidar registration and the
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
};

// The autopilot's own landing detector: any contact with the ground.
struct AutopilotGroundContact {
  bool detected{false};
};

} // namespace drone_city_nav
