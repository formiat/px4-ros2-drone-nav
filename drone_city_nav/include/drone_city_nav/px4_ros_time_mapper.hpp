#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace drone_city_nav {

struct Px4RosTimeMapperConfig {
  std::size_t min_samples{4U};
  std::size_t max_samples{64U};
  double min_scale{0.999};
  double max_scale{1.001};
  std::int64_t max_round_trip_time_ns{50'000'000};
};

struct Px4RosTimeMappingDiagnostics {
  bool ready{false};
  std::size_t sample_count{0U};
  double scale{1.0};
  double offset_ns{0.0};
  double min_observed_latency_ns{0.0};
  double max_fit_residual_ns{0.0};
  std::int64_t latest_estimated_offset_ns{0};
};

class Px4RosTimeMapper {
public:
  explicit Px4RosTimeMapper(const Px4RosTimeMapperConfig& config = {});

  void observeTimesync(std::uint64_t adjusted_timestamp_us,
                       std::int64_t estimated_offset_us,
                       std::uint32_t round_trip_time_us,
                       std::int64_t ros_receive_stamp_ns);

  [[nodiscard]] std::optional<std::int64_t>
  recoverPx4LocalTimeNs(std::uint64_t adjusted_timestamp_us) const noexcept;

  [[nodiscard]] std::optional<std::int64_t>
  px4LocalToRosTimeNs(std::int64_t px4_local_stamp_ns) const noexcept;

  [[nodiscard]] std::optional<std::int64_t>
  rosToPx4LocalTimeNs(std::int64_t ros_stamp_ns) const noexcept;

  [[nodiscard]] bool ready() const noexcept;
  [[nodiscard]] Px4RosTimeMappingDiagnostics diagnostics() const noexcept;
  void clear() noexcept;

private:
  struct Sample {
    std::int64_t px4_local_stamp_ns{0};
    std::int64_t ros_receive_stamp_ns{0};
  };

  void refit() noexcept;

  Px4RosTimeMapperConfig config_{};
  std::deque<Sample> samples_;
  double scale_{1.0};
  double offset_ns_{0.0};
  double min_observed_latency_ns_{0.0};
  double max_fit_residual_ns_{0.0};
  std::int64_t latest_estimated_offset_ns_{0};
  std::int64_t latest_estimated_offset_us_{0};
  bool offset_available_{false};
  bool ready_{false};
};

} // namespace drone_city_nav
