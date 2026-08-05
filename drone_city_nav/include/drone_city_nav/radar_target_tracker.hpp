#pragma once

#include "drone_city_nav/radar_model.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace drone_city_nav {

struct RadarOwnshipHistoryConfig {
  std::int64_t retention_ns{5'000'000'000LL};
  std::int64_t maximum_extrapolation_ns{100'000'000LL};
};

class RadarOwnshipHistory final {
public:
  explicit RadarOwnshipHistory(const RadarOwnshipHistoryConfig& config = {});

  bool add(const TimedVehicleState& state);
  [[nodiscard]] std::optional<TimedVehicleState>
  sample(std::int64_t stamp_ns) const noexcept;

  [[nodiscard]] std::size_t size() const noexcept {
    return samples_.size();
  }

private:
  void prune(std::int64_t newest_stamp_ns);

  RadarOwnshipHistoryConfig config_{};
  std::deque<TimedVehicleState> samples_;
};

struct RadarTargetTrackerConfig {
  double maximum_update_interval_s{4.0};
  double position_correction_gain{1.0};
  double velocity_correction_gain{0.5};
  double maximum_ownship_stamp_error_s{0.05};
  std::uint64_t track_id{1U};
};

struct RadarTrackEstimate {
  Point3 position{};
  Vec3 velocity{};
  std::int64_t stamp_ns{0};
  std::uint64_t track_id{0U};
  std::uint64_t source_scan_sequence{0U};
  std::uint64_t source_detection_id{0U};
  std::size_t measurement_count{0U};
  bool position_valid{false};
  bool velocity_valid{false};
};

class RadarTargetTracker final {
public:
  explicit RadarTargetTracker(const RadarTargetTrackerConfig& config = {});

  [[nodiscard]] RadarTrackEstimate update(const TimedVehicleState& ownship,
                                          const RadarDetectionSample& detection,
                                          std::int64_t measurement_stamp_ns,
                                          std::uint64_t scan_sequence);

  void reset() noexcept;

private:
  RadarTargetTrackerConfig config_{};
  std::optional<RadarTrackEstimate> previous_;
};

} // namespace drone_city_nav
