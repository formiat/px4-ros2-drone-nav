#include "drone_city_nav/px4_ros_time_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kNanosecondsPerMicrosecond{1000};

[[nodiscard]] std::optional<std::int64_t>
checkedAdjustedTimestampNs(const std::uint64_t adjusted_timestamp_us,
                           const std::int64_t estimated_offset_us) noexcept {
  std::uint64_t local_timestamp_us = adjusted_timestamp_us;
  if (estimated_offset_us >= 0) {
    const auto positive_offset_us = static_cast<std::uint64_t>(estimated_offset_us);
    if (local_timestamp_us >
        std::numeric_limits<std::uint64_t>::max() - positive_offset_us) {
      return std::nullopt;
    }
    local_timestamp_us += positive_offset_us;
  } else {
    const std::uint64_t magnitude_us =
        static_cast<std::uint64_t>(-(estimated_offset_us + 1)) + 1U;
    if (local_timestamp_us <= magnitude_us) {
      return std::nullopt;
    }
    local_timestamp_us -= magnitude_us;
  }
  const auto max_timestamp_us = static_cast<std::uint64_t>(
      std::numeric_limits<std::int64_t>::max() / kNanosecondsPerMicrosecond);
  if (local_timestamp_us == 0U || local_timestamp_us > max_timestamp_us) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(local_timestamp_us) * kNanosecondsPerMicrosecond;
}

[[nodiscard]] std::optional<std::int64_t>
roundedTimestamp(const double value_ns) noexcept {
  if (!std::isfinite(value_ns) || value_ns <= 0.0 ||
      value_ns > static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(std::llround(value_ns));
}

} // namespace

Px4RosTimeMapper::Px4RosTimeMapper(const Px4RosTimeMapperConfig& config)
    : config_{config} {
  config_.min_samples = std::max<std::size_t>(2U, config_.min_samples);
  config_.max_samples = std::max(config_.min_samples, config_.max_samples);
  if (!std::isfinite(config_.min_scale) || !std::isfinite(config_.max_scale) ||
      config_.min_scale <= 0.0 || config_.max_scale < config_.min_scale) {
    config_.min_scale = 0.999;
    config_.max_scale = 1.001;
  }
  config_.max_round_trip_time_ns =
      std::max<std::int64_t>(0, config_.max_round_trip_time_ns);
}

void Px4RosTimeMapper::observeTimesync(const std::uint64_t adjusted_timestamp_us,
                                       const std::int64_t estimated_offset_us,
                                       const std::uint32_t round_trip_time_us,
                                       const std::int64_t ros_receive_stamp_ns) {
  latest_estimated_offset_us_ = estimated_offset_us;
  if (estimated_offset_us >=
          std::numeric_limits<std::int64_t>::min() / kNanosecondsPerMicrosecond &&
      estimated_offset_us <=
          std::numeric_limits<std::int64_t>::max() / kNanosecondsPerMicrosecond) {
    latest_estimated_offset_ns_ = estimated_offset_us * kNanosecondsPerMicrosecond;
  } else {
    latest_estimated_offset_ns_ = 0;
  }
  offset_available_ = adjusted_timestamp_us > 0U;
  const auto px4_local_stamp_ns =
      checkedAdjustedTimestampNs(adjusted_timestamp_us, estimated_offset_us);
  const std::uint64_t round_trip_time_ns =
      static_cast<std::uint64_t>(round_trip_time_us) *
      static_cast<std::uint64_t>(kNanosecondsPerMicrosecond);
  if (!px4_local_stamp_ns.has_value() || ros_receive_stamp_ns <= 0 ||
      round_trip_time_ns > static_cast<std::uint64_t>(config_.max_round_trip_time_ns)) {
    return;
  }
  if (!samples_.empty() &&
      (*px4_local_stamp_ns <= samples_.back().px4_local_stamp_ns ||
       ros_receive_stamp_ns <= samples_.back().ros_receive_stamp_ns)) {
    return;
  }
  samples_.push_back(Sample{*px4_local_stamp_ns, ros_receive_stamp_ns});
  while (samples_.size() > config_.max_samples) {
    samples_.pop_front();
  }
  refit();
}

std::optional<std::int64_t> Px4RosTimeMapper::recoverPx4LocalTimeNs(
    const std::uint64_t adjusted_timestamp_us) const noexcept {
  if (!offset_available_ || adjusted_timestamp_us == 0U) {
    return std::nullopt;
  }
  return checkedAdjustedTimestampNs(adjusted_timestamp_us, latest_estimated_offset_us_);
}

std::optional<std::int64_t> Px4RosTimeMapper::px4LocalToRosTimeNs(
    const std::int64_t px4_local_stamp_ns) const noexcept {
  if (!ready_ || px4_local_stamp_ns <= 0) {
    return std::nullopt;
  }
  return roundedTimestamp(scale_ * static_cast<double>(px4_local_stamp_ns) +
                          offset_ns_);
}

std::optional<std::int64_t>
Px4RosTimeMapper::rosToPx4LocalTimeNs(const std::int64_t ros_stamp_ns) const noexcept {
  if (!ready_ || ros_stamp_ns <= 0 || !(scale_ > 0.0)) {
    return std::nullopt;
  }
  return roundedTimestamp((static_cast<double>(ros_stamp_ns) - offset_ns_) / scale_);
}

bool Px4RosTimeMapper::ready() const noexcept {
  return ready_;
}

Px4RosTimeMappingDiagnostics Px4RosTimeMapper::diagnostics() const noexcept {
  return Px4RosTimeMappingDiagnostics{
      .ready = ready_,
      .sample_count = samples_.size(),
      .scale = scale_,
      .offset_ns = offset_ns_,
      .min_observed_latency_ns = min_observed_latency_ns_,
      .max_fit_residual_ns = max_fit_residual_ns_,
      .latest_estimated_offset_ns = latest_estimated_offset_ns_,
  };
}

void Px4RosTimeMapper::clear() noexcept {
  samples_.clear();
  scale_ = 1.0;
  offset_ns_ = 0.0;
  min_observed_latency_ns_ = 0.0;
  max_fit_residual_ns_ = 0.0;
  latest_estimated_offset_ns_ = 0;
  latest_estimated_offset_us_ = 0;
  offset_available_ = false;
  ready_ = false;
}

void Px4RosTimeMapper::refit() noexcept {
  if (samples_.size() < 2U) {
    ready_ = false;
    return;
  }
  std::vector<double> pairwise_slopes;
  pairwise_slopes.reserve(samples_.size() * (samples_.size() - 1U) / 2U);
  for (std::size_t from = 0U; from + 1U < samples_.size(); ++from) {
    for (std::size_t to = from + 1U; to < samples_.size(); ++to) {
      const double delta_px4 = static_cast<double>(samples_[to].px4_local_stamp_ns -
                                                   samples_[from].px4_local_stamp_ns);
      const double delta_ros = static_cast<double>(samples_[to].ros_receive_stamp_ns -
                                                   samples_[from].ros_receive_stamp_ns);
      if (delta_px4 > 0.0 && std::isfinite(delta_ros)) {
        pairwise_slopes.push_back(delta_ros / delta_px4);
      }
    }
  }
  if (pairwise_slopes.empty()) {
    ready_ = false;
    return;
  }
  const auto median = pairwise_slopes.begin() +
                      static_cast<std::ptrdiff_t>(pairwise_slopes.size() / 2U);
  std::nth_element(pairwise_slopes.begin(), median, pairwise_slopes.end());
  scale_ = std::clamp(*median, config_.min_scale, config_.max_scale);

  // Callback transport latency is one-sided. The minimum residual is the best
  // available estimate of the clock offset without baking average queueing delay
  // into the acquisition-time mapping.
  offset_ns_ = std::numeric_limits<double>::infinity();
  for (const Sample& sample : samples_) {
    offset_ns_ = std::min(offset_ns_,
                          static_cast<double>(sample.ros_receive_stamp_ns) -
                              scale_ * static_cast<double>(sample.px4_local_stamp_ns));
  }
  min_observed_latency_ns_ = std::numeric_limits<double>::infinity();
  max_fit_residual_ns_ = 0.0;
  for (const Sample& sample : samples_) {
    const double residual =
        static_cast<double>(sample.ros_receive_stamp_ns) -
        (scale_ * static_cast<double>(sample.px4_local_stamp_ns) + offset_ns_);
    min_observed_latency_ns_ = std::min(min_observed_latency_ns_, residual);
    max_fit_residual_ns_ = std::max(max_fit_residual_ns_, std::abs(residual));
  }
  ready_ = samples_.size() >= config_.min_samples && std::isfinite(offset_ns_);
}

} // namespace drone_city_nav
