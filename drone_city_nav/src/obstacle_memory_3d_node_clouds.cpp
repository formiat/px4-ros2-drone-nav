#include "drone_city_nav/autopilot_state.hpp"
#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/cooperative_traffic_ros.hpp"
#include "drone_city_nav/dynamic_agent_lidar_state.hpp"
#include "drone_city_nav/latest_processing_queue.hpp"
#include "drone_city_nav/latest_sensor_obstacle_scan.hpp"
#include "drone_city_nav/latest_sensor_obstacle_scan_ros.hpp"
#include "drone_city_nav/lidar_acquisition_pose.hpp"
#include "drone_city_nav/lidar_debug_pointclouds.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/lidar_scan_3d.hpp"
#include "drone_city_nav/lidar_scan_patterns_3d.hpp"
#include "drone_city_nav/lidar_self_filter.hpp"
#include "drone_city_nav/msg/cooperative_flight_intent.hpp"
#include "drone_city_nav/msg/latest_sensor_obstacle_scan.hpp"
#include "drone_city_nav/msg/spectator_target.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory_3d.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/spectator_diagnostics_selection.hpp"
#include "drone_city_nav/spectator_diagnostics_selection_ros.hpp"
#include "drone_city_nav/tracked_agent_lidar_filter.hpp"
#include "drone_city_nav/transport_latency_ros.hpp"
#include "drone_city_nav/visualization_marker_helpers.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "obstacle_memory_3d_node.hpp"
#include "obstacle_memory_3d_worker.hpp"
#include "obstacle_memory_node_helpers.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool volumeContains(const DynamicAgentLidarVolume& volume,
                                  const Point3& point) noexcept {
  return std::isfinite(volume.position.x) && std::isfinite(volume.position.y) &&
         std::isfinite(volume.position.z) && volume.radius_m > 0.0 &&
         point.z >= volume.position.z - volume.lower_extent_m &&
         point.z <= volume.position.z + volume.upper_extent_m &&
         std::hypot(point.x - volume.position.x, point.y - volume.position.y) <=
             volume.radius_m;
}

[[nodiscard]] bool
anyVolumeContains(const std::span<const DynamicAgentLidarVolume> volumes,
                  const Point3& point) noexcept {
  return std::ranges::any_of(
      volumes, [&point](const auto& volume) { return volumeContains(volume, point); });
}

} // namespace

// One scan: aligned to the pose history, projected, filtered, published as
// the latest sensor scan and handed to the memory worker.

PendingPointCloudDisposition
ObstacleMemory3DNode::processPendingCloud(const PendingPointCloud3D& pending) {
  const auto processing_started = std::chrono::steady_clock::now();
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const std::optional<std::int64_t> scan_stamp_ns =
      validRosStampNanoseconds(pending.cloud.header.stamp);
  const LaserScanTiming timing{
      .first_beam_stamp_ns = scan_stamp_ns.value_or(0),
      .first_beam_stamp_valid = scan_stamp_ns.has_value(),
      .time_increment_s = 0.0,
      .receive_stamp_ns = pending.receive_stamp_ns,
      .receive_stamp_valid = pending.receive_stamp_ns > 0,
  };
  LidarAcquisitionPoseResult acquisition;
  std::uint64_t pose_generation{0U};
  {
    const std::scoped_lock pose_lock{pose_history_mutex_};
    acquisition = resolveLidarAcquisitionBeamPoses(
        lidar_pose_history_, timing, 1U, acquisition_pose_config_,
        use_px4_heading_for_scan_ ? std::nullopt
                                  : std::optional<double>{initial_heading_rad_},
        &time_mapper_);
    pose_generation = lidar_pose_history_.generation();
  }
  const bool permanent_failure =
      acquisition.status == LidarAcquisitionPoseStatus::kInvalidSourceTimeOffset ||
      acquisition.status == LidarAcquisitionPoseStatus::kInvalidScanTimestamp;
  const bool wait_expired =
      pending.receive_stamp_ns <= 0 ||
      now_ns - pending.receive_stamp_ns >= alignment_maximum_wait_ns_;
  const std::string alignment_diagnostic = formatLidarAcquisitionPoseDiagnostic(
      acquisition.resolved() ? "3D lidar acquisition pose"
                             : "3D lidar acquisition pose rejected",
      acquisition, timing, now_ns);
  if (!acquisition.resolved()) {
    if (!permanent_failure && !wait_expired) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                           "LIDAR3D_ALIGNMENT waiting=true queue_wait_ms=%.3f %s",
                           1.0e-6 *
                               static_cast<double>(now_ns - pending.receive_stamp_ns),
                           alignment_diagnostic.c_str());
      return PendingPointCloudDisposition::kWaitForPoseBracket;
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "LIDAR3D_ALIGNMENT dropped=true queue_wait_ms=%.3f %s",
                         1.0e-6 *
                             static_cast<double>(now_ns - pending.receive_stamp_ns),
                         alignment_diagnostic.c_str());
    return PendingPointCloudDisposition::kConsumed;
  }
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                       "LIDAR3D_ALIGNMENT dropped=false queue_wait_ms=%.3f %s %s",
                       1.0e-6 * static_cast<double>(now_ns - pending.receive_stamp_ns),
                       transportLatencyFields(cloud_delivery_ms_).c_str(),
                       alignment_diagnostic.c_str());
  const std::optional<std::vector<Point3>> raw_returns =
      decodePointCloudReturns(pending.cloud);
  if (!raw_returns.has_value()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "LIDAR3D_SCAN rejected=true reason=invalid_pointcloud");
    return PendingPointCloudDisposition::kConsumed;
  }
  OrganizedLidarScan3DResult decoded =
      hit_only_returns_
          ? decodeHitOnlyReturns3D(
                *raw_returns,
                decodePointCloudFloatField(pending.cloud, "intensity")
                    .value_or(std::vector<float>{}),
                scan_config_.minimum_range_m, scan_config_.maximum_range_m)
          : decodeOrganizedLidarScan3D(*raw_returns, scan_config_);
  // Roadmap item 18 stage 0: the patterns in which the lidar fails, read
  // from the organized scan; an aerosol's returns and a dropout bundle's
  // rays become misses to their range with fail evidence there.
  LidarScanPatternsResult patterns;
  if (!hit_only_returns_ && decoded.organized_dimensions_match) {
    patterns = classifyLidarScanPatterns3D(
        decoded,
        decodePointCloudFloatField(pending.cloud, "intensity")
            .value_or(std::vector<float>{}),
        scan_config_, lidar_patterns_config_);
  }
  if (!decoded.organized_dimensions_match) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "LIDAR3D_SCAN rejected=true reason=beam_geometry_mismatch width=%u "
        "height=%u points=%zu expected=%zu",
        pending.cloud.width, pending.cloud.height, raw_returns->size(),
        scan_config_.horizontal_samples * scan_config_.vertical_samples);
    return PendingPointCloudDisposition::kConsumed;
  }

  const LidarProjectionPose& pose = acquisition.alignment.poses.front();
  const LidarProjectionBodyFrame body_frame =
      lidarProjectionBodyFrame(pose, projection_config_);
  if (!body_frame.valid) {
    return PendingPointCloudDisposition::kConsumed;
  }
  const std::int64_t acquisition_stamp_ns = timing.first_beam_stamp_ns;
  DynamicAgentLidarFilterPlan filter_plan =
      dynamic_agent_state_->makeFilterPlan(now_ns, acquisition_stamp_ns);
  // Surface samples reconstructed between adjacent returns pass through the
  // same projection and filters as measured returns and enter the persistent
  // memory as occupied evidence without free-space evidence. The latest
  // obstacle scan stays the measured returns alone: its contract bounds the
  // hit count by the beam count.
  // A sensor that answers only where it measured interpolates nothing; what
  // it could not see enters the memory the same way (roadmap item 17 stage
  // 8).
  std::vector<LidarBeamSample3D> surface_samples =
      hit_only_returns_
          ? decoded.unobservable
          : interpolateOrganizedLidarSurfaces3D(decoded.beams, scan_config_,
                                                surface_interpolation_config_);
  if (!hit_only_returns_) {
    surface_samples.insert(surface_samples.end(), decoded.unobservable.begin(),
                           decoded.unobservable.end());
  }
  std::vector<LidarBeam3D> memory_beams;
  std::vector<Point3> hit_points_map;
  std::vector<Point3> hit_points_body;
  memory_beams.reserve(decoded.beams.size() + surface_samples.size());
  hit_points_map.reserve(decoded.hit_beams);
  hit_points_body.reserve(decoded.hit_beams);
  Point3 ray_origin{};
  bool origin_valid{false};
  std::size_t cooperative_filtered{0U};
  std::size_t self_filtered{0U};
  std::size_t persistent_self_filtered{0U};
  std::size_t projection_invalid{0U};
  std::size_t unobservable_on_path{0U};
  const auto process_sample = [&](const LidarBeamSample3D& sample) {
    const LidarRayProjection3D ray =
        projectLidarRay3D(pose, projection_config_, sample.direction_lidar_flu);
    LidarBeam3D beam{.direction_map = ray.direction_map,
                     .range_m = sample.range_m,
                     .hit = sample.hit,
                     .valid = sample.valid && ray.valid,
                     .surface_only = sample.interpolated};
    if (!beam.valid) {
      ++projection_invalid;
      memory_beams.push_back(beam);
      return;
    }
    ray_origin = ray.origin_map_m;
    origin_valid = true;
    if (beam.hit) {
      const Point3 endpoint{ray.origin_map_m.x + beam.range_m * beam.direction_map.x,
                            ray.origin_map_m.y + beam.range_m * beam.direction_map.y,
                            ray.origin_map_m.z + beam.range_m * beam.direction_map.z};
      // The way the vehicle came is never observed unobservable
      // (specification K14, I3's exit guarantee): a pair that turns back
      // at the dim edge of a dark looks into the same dark and would close it.
      if (hit_only_returns_ && sample.interpolated &&
          std::ranges::any_of(flown_path_, [&endpoint](const Point3& pose) {
            return squaredDistance(pose, endpoint) <=
                   kFlownPathClearanceM * kFlownPathClearanceM;
          })) {
        ++unobservable_on_path;
        return;
      }
      const Point3 endpoint_body = lidarMapPointToBody(body_frame, endpoint);
      std::optional<Point3> occupancy_voxel_center_body;
      const std::optional<GridIndex3D> occupancy_cell =
          grid_geometry_.worldToCell(endpoint);
      if (occupancy_cell.has_value()) {
        occupancy_voxel_center_body =
            lidarMapPointToBody(body_frame, grid_geometry_.cellCenter(*occupancy_cell));
      }
      const LidarSelfFilterDisposition self_disposition = classifyLidarSelfHit(
          endpoint_body, occupancy_voxel_center_body, self_filter_config_);
      if (self_disposition ==
          LidarSelfFilterDisposition::kDiscardFromAllObstacleInputs) {
        beam.valid = false;
        ++self_filtered;
        memory_beams.push_back(beam);
        return;
      }
      const bool cooperative_peer =
          anyVolumeContains(filter_plan.cooperative_memory_exclusions, endpoint);
      if (cooperative_peer) {
        beam.valid = false;
        ++cooperative_filtered;
      } else {
        if (!sample.interpolated) {
          hit_points_map.push_back(endpoint);
          hit_points_body.push_back(endpoint_body);
        }
        if (self_disposition ==
            LidarSelfFilterDisposition::kDiscardFromPersistentMemory) {
          beam.valid = false;
          ++persistent_self_filtered;
        }
      }
    }
    memory_beams.push_back(beam);
  };
  for (const LidarBeamSample3D& sample : decoded.beams) {
    process_sample(sample);
  }
  for (const LidarBeamSample3D& sample : surface_samples) {
    process_sample(sample);
  }
  if (!origin_valid) {
    return PendingPointCloudDisposition::kConsumed;
  }
  if (flown_path_.empty() || squaredDistance(flown_path_.back(), ray_origin) >=
                                 kFlownPathSpacingM * kFlownPathSpacingM) {
    flown_path_.push_back(ray_origin);
    if (flown_path_.size() > kFlownPathPoses) {
      flown_path_.pop_front();
    }
  }

  std_msgs::msg::Header source_header = pending.cloud.header;
  LatestSensorObstacleScanBuildResult latest;
  latest.acquisition_body_frame = body_frame;
  latest.hit_points_body_frd =
      thinnedNearestReturns(hit_points_body, kLatestSensorScanCellM);
  latest.source_beam_count = decoded.beams.size();
  const std::size_t dynamic_filtered = cooperative_filtered;
  latest.invalid_beam_count = projection_invalid + dynamic_filtered + self_filtered;
  latest.valid = true;
  latest_scan_pub_->publish(makeLatestSensorObstacleScanMessage(
      latest, source_header, frame_id_, acquisition_stamp_ns,
      latest_scan_producer_instance_id_, ++latest_scan_sequence_, pose_generation));
  const bool publish_current_cloud =
      persistent_memory_diagnostics_enabled_ && persistent_memory_selection_.selected();
  if (publish_current_cloud) {
    current_returns_pub_->publish(buildLidarDebugPointCloud(
        hit_points_map, rclcpp::Time{acquisition_stamp_ns, RCL_ROS_TIME}, frame_id_,
        gazebo_aligned_rviz_axes_swapped_));
  }

  bool memory_scan_dropped{false};
  if (memory_worker_) {
    memory_scan_dropped = memory_worker_->enqueue(PersistentLidarScan3D{
        .origin_map = ray_origin,
        .beams = std::move(memory_beams),
        .dynamic_filter_plan = std::move(filter_plan),
        .acquisition_stamp_ns = acquisition_stamp_ns,
        .enqueued_at = std::chrono::steady_clock::now(),
        .altitude_m = pose.altitude_m,
        .source_beams = decoded.beams.size(),
        .projection_invalid = projection_invalid,
        .self_filtered = self_filtered,
        .persistent_self_filtered = persistent_self_filtered,
        .unobservable_on_path = unobservable_on_path,
        .aerosol_beams = patterns.aerosol_beams,
        .thin_aerosol_beams = patterns.thin_aerosol_beams,
        .dropout_beams = patterns.dropout_beams,
        .cooperative_filtered = cooperative_filtered,
        .altitude_valid = pose.altitude_valid,
        .publish_debug = publish_current_cloud,
    });
  }
  const auto processing_finished = std::chrono::steady_clock::now();
  const double processing_ms = std::chrono::duration<double, std::milli>(
                                   processing_finished - processing_started)
                                   .count();
  const double acquisition_age_ms =
      1.0e-6 *
      static_cast<double>(get_clock()->now().nanoseconds() - acquisition_stamp_ns);
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "LIDAR3D_CURRENT_SCAN accepted=true stamp_ns=%" PRId64 " sequence=%" PRIu64
      " source=%zu hits=%zu surface_samples=%zu invalid=%zu "
      "acquisition_age_ms=%.3f processing_ms=%.3f alignment_coalesced=%" PRIu64
      " memory_scan_dropped=%s debug=%s",
      acquisition_stamp_ns, latest_scan_sequence_, decoded.beams.size(),
      hit_points_body.size(), surface_samples.size(), latest.invalid_beam_count,
      acquisition_age_ms, processing_ms, alignment_coalesced_clouds_,
      memory_scan_dropped ? "true" : "false", publish_current_cloud ? "true" : "false");
  return PendingPointCloudDisposition::kConsumed;
}

} // namespace drone_city_nav
