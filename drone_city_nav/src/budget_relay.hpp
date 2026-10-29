// The onboard budget as a relay (roadmap item 36, specification K53): the
// outputs of the onboard loop the relay carries and the ages, measured on the
// host of record, it holds each of them to; and the stamps read off the
// serialized bytes. A simulation component's vocabulary, private to the relay
// and its test; no production node includes it (the contract test holds it there).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string_view>

namespace drone_city_nav::budget_relay {

inline constexpr std::string_view kBudgetedSuffix = "/budgeted";
inline constexpr std::string_view kUnbudgetedOdometry =
    "/drone_city_nav/vehicle_visual_odometry/unbudgeted";
inline constexpr std::string_view kOdometry = "/fmu/in/vehicle_visual_odometry";

enum class Qos : std::uint8_t {
  kSensor,              // best effort, volatile, keep last 5
  kBestEffortTransient, // the memory's delta: best effort, transient local, keep last 1
  kReliableTransient,   // the memory's snapshot: reliable, transient local, keep last 1
  kReliable2,           // the planner's horizon: reliable, volatile, keep last 2
  kBestEffort10         // the odometry: best effort, volatile, keep last 10
};

struct Relayed {
  std::string_view name;
  std::string_view source; // the topic the producer publishes to
  std::string_view target; // the topic the consumer reads
  std::string_view type;   // "package/msg/Type"
  Qos qos;
  bool header_first; // a std_msgs/Header leads the message; else PX4's timestamp_sample
  double stereo_age_ms; // the median age at the consumer on the stereo set, 0 when not
                        // carried
  double lidar_age_ms;  // the same on the lidar set
};

// The ages at the consumer the relay measured itself on the host of record
// (K53: r2438 stereo at a real-time factor of 0.82, r2439 lidar at 1.00, the
// medians of its 10 s rows). An output stamped at its observation carries the
// whole chain's latency; one stamped at its publication (the planner's
// horizon, the camera estimator's odometry at the IMU's time) the transport
// alone, and its node's duration is the step controller's to pay.
inline constexpr std::array<Relayed, 6> kRelayed{{
    {"depth_points", "/stereo_depth/points", "/stereo_depth/points/budgeted",
     "sensor_msgs/msg/PointCloud2", Qos::kSensor, true, 154.0, 0.0},
    {"odometry", kUnbudgetedOdometry, kOdometry, "px4_msgs/msg/VehicleOdometry",
     Qos::kBestEffort10, false, 6.4, 58.0},
    {"memory_delta", "/drone_city_nav/raw_obstacle_delta_3d",
     "/drone_city_nav/raw_obstacle_delta_3d/budgeted",
     "drone_city_nav/msg/RawObstacleDelta3D", Qos::kBestEffortTransient, true, 210.0,
     160.0},
    {"memory_snapshot", "/drone_city_nav/raw_obstacle_snapshot_3d",
     "/drone_city_nav/raw_obstacle_snapshot_3d/budgeted",
     "drone_city_nav/msg/RawObstacleSnapshot3D", Qos::kReliableTransient, true, 202.0,
     172.0},
    {"latest_scan", "/drone_city_nav/latest_sensor_obstacle_scan",
     "/drone_city_nav/latest_sensor_obstacle_scan/budgeted",
     "drone_city_nav/msg/LatestSensorObstacleScan", Qos::kSensor, true, 170.0, 68.0},
    {"horizon", "/drone_city_nav/mppi/execution_horizon",
     "/drone_city_nav/mppi/execution_horizon/budgeted",
     "drone_city_nav/msg/MppiTrajectoryHorizon", Qos::kReliable2, true, 4.0, 4.0},
}};

// The message's stamp in nanoseconds read off its serialized bytes: after the
// 4-byte CDR encapsulation a leading std_msgs/Header starts with int32 seconds
// and uint32 nanoseconds; PX4's VehicleOdometry with uint64 timestamp then
// uint64 timestamp_sample, microseconds. Nothing when the buffer is too short.
inline std::optional<std::int64_t> stampNsOf(const std::uint8_t* data, std::size_t size,
                                             bool header_first) noexcept {
  if (header_first) {
    if (size < 12U) {
      return std::nullopt;
    }
    std::int32_t seconds = 0;
    std::uint32_t nanoseconds = 0;
    std::memcpy(&seconds, data + 4, sizeof(seconds));
    std::memcpy(&nanoseconds, data + 8, sizeof(nanoseconds));
    return static_cast<std::int64_t>(seconds) * 1'000'000'000LL + nanoseconds;
  }
  if (size < 20U) {
    return std::nullopt;
  }
  std::uint64_t sample_us = 0;
  std::memcpy(&sample_us, data + 12, sizeof(sample_us));
  return static_cast<std::int64_t>(sample_us) * 1'000LL;
}

} // namespace drone_city_nav::budget_relay
