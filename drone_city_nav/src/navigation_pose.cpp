#include "drone_city_nav/navigation_pose.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace drone_city_nav {

MappingYawTracker::MappingYawTracker(const bool use_px4_heading,
                                     const double initial_map_heading_rad,
                                     const std::size_t stable_sample_count,
                                     const double maximum_sample_delta_rad) noexcept
    : use_px4_heading_{use_px4_heading},
      initial_map_heading_rad_{normalizeYaw(initial_map_heading_rad)},
      required_stable_sample_count_{std::max<std::size_t>(1U, stable_sample_count)},
      maximum_sample_delta_rad_{
          std::isfinite(maximum_sample_delta_rad) && maximum_sample_delta_rad >= 0.0
              ? std::min(maximum_sample_delta_rad, std::numbers::pi)
              : 0.05} {
}

MappingYawSelection MappingYawTracker::update(const bool px4_heading_ready,
                                              const double px4_heading_rad) noexcept {
  if (!use_px4_heading_) {
    if (!std::isfinite(initial_map_heading_rad_)) {
      return {};
    }
    return MappingYawSelection{initial_map_heading_rad_,
                               MappingYawSource::kInitialMapHeading, true};
  }
  if (!px4_heading_ready || !std::isfinite(px4_heading_rad)) {
    reset();
    return {};
  }

  const double normalized_heading = normalizeYaw(px4_heading_rad);
  if (px4_stable_) {
    previous_px4_heading_rad_ = normalized_heading;
    return MappingYawSelection{normalized_heading, MappingYawSource::kPx4Heading, true};
  }
  const bool stable_with_previous =
      previous_px4_heading_rad_.has_value() &&
      std::abs(normalizeYaw(normalized_heading - *previous_px4_heading_rad_)) <=
          maximum_sample_delta_rad_;
  stable_sample_count_ = stable_with_previous ? stable_sample_count_ + 1U : 1U;
  previous_px4_heading_rad_ = normalized_heading;
  px4_stable_ = stable_sample_count_ >= required_stable_sample_count_;
  if (!px4_stable_) {
    return {};
  }
  return MappingYawSelection{normalized_heading, MappingYawSource::kPx4Heading, true};
}

bool MappingYawTracker::px4Stable() const noexcept {
  return px4_stable_;
}

std::size_t MappingYawTracker::stableSampleCount() const noexcept {
  return stable_sample_count_;
}

void MappingYawTracker::reset() noexcept {
  previous_px4_heading_rad_.reset();
  stable_sample_count_ = 0U;
  px4_stable_ = false;
}

double normalizeYaw(const double yaw_rad) noexcept {
  if (!std::isfinite(yaw_rad)) {
    return yaw_rad;
  }

  double normalized = std::remainder(yaw_rad, 2.0 * std::numbers::pi);
  if (normalized <= -std::numbers::pi) {
    normalized += 2.0 * std::numbers::pi;
  }
  if (normalized > std::numbers::pi) {
    normalized -= 2.0 * std::numbers::pi;
  }
  return normalized;
}

const char* mappingYawSourceName(const MappingYawSource source) noexcept {
  switch (source) {
    case MappingYawSource::kUnavailable:
      return "unavailable";
    case MappingYawSource::kInitialMapHeading:
      return "initial_map_heading";
    case MappingYawSource::kPx4Heading:
      return "px4_heading";
  }
  return "unknown";
}

bool px4HeadingReadyForMapping(const bool heading_good_for_control,
                               const double heading_rad,
                               const double heading_variance_rad2,
                               const double maximum_heading_variance_rad2) noexcept {
  return heading_good_for_control && std::isfinite(heading_rad) &&
         std::isfinite(heading_variance_rad2) && heading_variance_rad2 >= 0.0 &&
         std::isfinite(maximum_heading_variance_rad2) &&
         maximum_heading_variance_rad2 >= 0.0 &&
         heading_variance_rad2 <= maximum_heading_variance_rad2;
}

bool timestampIsFresh(const std::int64_t stamp_ns, const std::int64_t now_ns,
                      const std::int64_t max_staleness_ns,
                      const std::int64_t max_future_skew_ns) noexcept {
  if (max_staleness_ns <= 0) {
    return true;
  }
  if (stamp_ns <= 0 || now_ns <= 0) {
    return false;
  }
  if (stamp_ns > now_ns) {
    return max_future_skew_ns >= 0 && stamp_ns - now_ns <= max_future_skew_ns;
  }
  return now_ns - stamp_ns <= max_staleness_ns;
}

void invalidateNavigationPose(NavigationPose2D& pose) noexcept {
  pose = NavigationPose2D{};
}

bool navigationPoseReadyForScan(const NavigationPose2D& pose,
                                const std::int64_t last_update_ns,
                                const std::int64_t now_ns,
                                const std::int64_t max_staleness_ns) noexcept {
  return pose.position_valid && pose.yaw_valid && std::isfinite(pose.pose.position.x) &&
         std::isfinite(pose.pose.position.y) && std::isfinite(pose.pose.yaw_rad) &&
         timestampIsFresh(last_update_ns, now_ns, max_staleness_ns);
}

std::optional<NavigationPose2D>
makeNavigationPoseFromPx4LocalPosition(const Px4LocalPositionSample& sample,
                                       const Px4LocalPoseConfig& config) noexcept {
  if (!sample.xy_valid || !std::isfinite(sample.x_m) || !std::isfinite(sample.y_m)) {
    return std::nullopt;
  }

  const Px4MapFrameTransform transform{
      .map_origin =
          Point3{config.map_origin_x_m, config.map_origin_y_m, config.map_origin_z_m},
      .m00 = config.px4_to_map_m00,
      .m01 = config.px4_to_map_m01,
      .m10 = config.px4_to_map_m10,
      .m11 = config.px4_to_map_m11,
  };
  NavigationPose2D pose{};
  pose.pose.position = transform.localPositionToMap(Point2{sample.x_m, sample.y_m});
  pose.stamp_ns = sample.stamp_ns;
  pose.position_valid = true;

  if (sample.z_valid && std::isfinite(sample.z_m)) {
    pose.altitude_m = -sample.z_m + transform.map_origin.z;
    pose.altitude_valid = true;
  }

  if (config.use_heading_for_yaw) {
    if (sample.heading_good_for_control && std::isfinite(sample.heading_rad)) {
      pose.pose.yaw_rad =
          normalizeYaw(transform.px4HeadingToMapYaw(sample.heading_rad));
      pose.yaw_valid = true;
    }
    return pose;
  }

  if (std::isfinite(config.initial_heading_rad)) {
    pose.pose.yaw_rad = normalizeYaw(config.initial_heading_rad);
    pose.yaw_valid = true;
  }
  return pose;
}

Px4LocalPoseUpdateStatus
updateNavigationPoseFromPx4LocalPosition(const Px4LocalPositionSample& sample,
                                         const Px4LocalPoseConfig& config,
                                         NavigationPose2D& state) noexcept {
  const auto pose = makeNavigationPoseFromPx4LocalPosition(sample, config);
  if (!pose.has_value()) {
    invalidateNavigationPose(state);
    return Px4LocalPoseUpdateStatus::kInvalidPosition;
  }

  if (!pose->yaw_valid) {
    invalidateNavigationPose(state);
    return Px4LocalPoseUpdateStatus::kInvalidYaw;
  }

  state = *pose;
  return Px4LocalPoseUpdateStatus::kAccepted;
}

} // namespace drone_city_nav
