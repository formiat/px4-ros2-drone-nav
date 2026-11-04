#pragma once
// The camera estimator's node class: the members and the short methods; its
// constructor is visual_inertial_odometry_node.cpp's, its depth against the
// estimator's map visual_inertial_odometry_node_map.cpp's (split by theme on
// 2026-11-04, the source size contract).

#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/lidar_acquisition_pose.hpp"
#include "drone_city_nav/px4_autopilot_adapter.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"
#include "drone_city_nav/step_acknowledger.hpp"
#include "drone_city_nav/visual_inertial_odometry.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "stereo_calibration_ros.hpp"
#include "stereo_feature_tracker.hpp"
#include "stereo_map_registration.hpp"
#include "visual_inertial_gnss_input.hpp"
#include "visual_inertial_odometry_node_internal.hpp"

namespace drone_city_nav::visual_inertial_odometry_node_internal {

class VisualInertialOdometryNode final : public rclcpp::Node {
public:
  // The parameters, the subscriptions and the estimator
  // (visual_inertial_odometry_node.cpp).
  explicit VisualInertialOdometryNode(const rclcpp::NodeOptions& options);

private:
  struct Pair {
    sensor_msgs::msg::Image::ConstSharedPtr left;
    sensor_msgs::msg::Image::ConstSharedPtr right;
    std::int64_t stamp_ns{0};
  };

  struct GyroSample {
    std::int64_t stamp_ns{0};
    Eigen::Vector3d rate_radps{Eigen::Vector3d::Zero()};
  };

  void onImu(const AutopilotImuSample& sample) {
    const std::scoped_lock lock{odometry_mutex_};
    const LidarPoseSourceStampResult source = resolveLidarPoseSourceStamp(
        time_mapper_, sample.timestamp_us, get_clock()->now().nanoseconds());
    if (!source.resolved()) {
      ++unmapped_imu_samples_;
      return;
    }
    const std::int64_t stamp_ns = source.mapped_ros_stamp_ns;
    const bool hole =
        last_imu_stamp_ns_ > 0 && stamp_ns - last_imu_stamp_ns_ > maximum_unaided_ns_;
    if (last_imu_stamp_ns_ > 0 && stamp_ns > last_imu_stamp_ns_) {
      imu_gap_max_ns_ = std::max(imu_gap_max_ns_, stamp_ns - last_imu_stamp_ns_);
    }
    last_imu_stamp_ns_ = std::max(last_imu_stamp_ns_, stamp_ns);
    if (gnss_input_ != nullptr) {
      if (const std::optional<GnssTrustVerdict> gone =
              gnss_input_->expire(get_clock()->now().nanoseconds())) {
        declareGnssTrust(get_logger(), *mission_event_pub_, *gone, 0.0,
                         gnss_trusted_declared_);
      }
    }
    const Eigen::Vector3d gyro{sample.gyro_radps.x, sample.gyro_radps.y,
                               sample.gyro_radps.z};
    odometry_->addImu(VisualInertialImuSample{
        .stamp_ns = stamp_ns,
        .gyro_radps = gyro,
        .accelerometer_mps2 =
            Eigen::Vector3d{sample.accelerometer_mps2.x, sample.accelerometer_mps2.y,
                            sample.accelerometer_mps2.z}});
    gyro_.push_back(GyroSample{.stamp_ns = stamp_ns, .rate_radps = gyro});
    ++imu_samples_;
    // Specification K28: the stream back after a hole. The frames held
    // through it could only meet a lost filter, a second and a half of them
    // under this lock, which kept the restart waiting (r1306); they are
    // dropped, the next frame tracks without the gyroscope's turn across the
    // hole, and the filter starts again here. Only a hole restarts it: one
    // lost after its dead reckoning stays lost (specification K13).
    if (hole && odometry_->initialized()) {
      std::erase_if(pending_,
                    [stamp_ns](const Pair& pair) { return pair.stamp_ns <= stamp_ns; });
      previous_frame_stamp_ns_ = 0;
      if (aided_) {
        restart_due_ = true;
      } else {
        startAfresh("imu_hole");
      }
    }
    if (restart_due_ && reinitialiseFromAutopilot()) {
      restart_due_ = false;
    }
    // A frame waits for the IMU up to its own stamp, in order: the filter
    // clones the pose where the propagation stands.
    while (!pending_.empty() && last_imu_stamp_ns_ >= pending_.front().stamp_ns) {
      const Pair pair = std::move(pending_.front());
      pending_.pop_front();
      onPair(pair);
    }
    publishToAutopilot(stamp_ns);
  }

  // The autopilot receives the estimate every 40 ms, carried from the last
  // frame through the IMU: at the pair's 7.5 Hz alone one late frame opens
  // the 200 ms without odometry that end its external-vision fusion, and
  // restarting that fusion reset the height by a metre and more in roadmap
  // item 13 (r389, r393, r398). The stamp is the pose's own moment in the
  // autopilot's synchronised clock, as the lidar-inertial node dates its.
  void publishToAutopilot(const std::int64_t stamp_ns) {
    if (!publish_to_autopilot_ || !odometry_->initialized() ||
        stamp_ns - last_autopilot_stamp_ns_ < kAutopilotOdometryPeriodNs) {
      return;
    }
    VisualInertialEstimate estimate = odometry_->predicted(stamp_ns);
    applyMapFrame(estimate, stamp_ns);
    const std::optional<std::int64_t> px4_local_ns =
        time_mapper_.rosToPx4LocalTimeNs(estimate.stamp_ns);
    if ((!estimate.healthy && !estimate.dead_reckoning && !estimate.in_void) ||
        estimate.stamp_ns != stamp_ns || !px4_local_ns.has_value()) {
      return;
    }
    // The void (roadmap item 31, K46) is declared, as dead reckoning is; the
    // autopilot receives the pose as a sound one (quality 100): the vehicle
    // flies on through an emptiness measured, and the offboard's descent
    // (K19) stays the failed light's.
    if (estimate.in_void != void_declared_) {
      RCLCPP_WARN(get_logger(),
                  "VISUAL_INERTIAL_ODOMETRY_VOID started=%s unaided_s=%.2f",
                  estimate.in_void ? "true" : "false", estimate.unaided_s);
      std_msgs::msg::String event;
      event.data = std::string{"VISUAL_INERTIAL_ODOMETRY_VOID started="} +
                   (estimate.in_void ? "true" : "false");
      mission_event_pub_->publish(event);
      void_declared_ = estimate.in_void;
    }
    // Dead reckoning (the ladder's second rung, roadmap item 17 stage 5) is
    // declared; the filter holds its height through it.
    if (estimate.dead_reckoning) {
      if (!dead_reckoning_declared_) {
        RCLCPP_WARN(get_logger(),
                    "VISUAL_INERTIAL_ODOMETRY_DEAD_RECKONING started=true "
                    "unaided_s=%.2f",
                    estimate.unaided_s);
      }
    } else if (dead_reckoning_declared_) {
      RCLCPP_WARN(get_logger(),
                  "VISUAL_INERTIAL_ODOMETRY_DEAD_RECKONING started=false");
    }
    if (estimate.dead_reckoning != dead_reckoning_declared_) {
      std_msgs::msg::String event;
      event.data = std::string{"VISUAL_INERTIAL_ODOMETRY_DEAD_RECKONING started="} +
                   (estimate.dead_reckoning ? "true" : "false");
      mission_event_pub_->publish(event);
    }
    dead_reckoning_declared_ = estimate.dead_reckoning;
    const std::int64_t synchronised_ns =
        *px4_local_ns - time_mapper_.diagnostics().latest_estimated_offset_ns;
    if (synchronised_ns <= 0) {
      return;
    }
    px4_msgs::msg::VehicleOdometry odometry = px4VisualOdometryFromEstimate(
        estimate, static_cast<std::uint64_t>(synchronised_ns / 1000));
    if (estimate.in_void) {
      odometry.quality = 100;
    }
    odometry_pub_->publish(odometry);
    last_autopilot_stamp_ns_ = stamp_ns;
    ++published_poses_;
    // How old the pose is when it leaves for the autopilot: a host that
    // starves the estimator shows here and nowhere else.
    pose_age_max_ns_ =
        std::max(pose_age_max_ns_, get_clock()->now().nanoseconds() - stamp_ns);
  }

  void queueIfPaired() {
    if (left_ == nullptr || right_ == nullptr ||
        left_->header.stamp != right_->header.stamp) {
      return;
    }
    Pair pair{.left = std::move(left_), .right = std::move(right_), .stamp_ns = 0};
    left_.reset();
    right_.reset();
    pair.stamp_ns =
        rclcpp::Time{pair.left->header.stamp}.nanoseconds() + frame_stamp_offset_ns_;
    const auto usable = [this](const sensor_msgs::msg::Image& image) {
      return image.encoding == "mono8" && image.width == image_width_ &&
             image.height == image_height_ &&
             image.data.size() >= static_cast<std::size_t>(image.step) * image.height;
    };
    if (!usable(*pair.left) || !usable(*pair.right)) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "VISUAL_INERTIAL_ODOMETRY rejected=true reason=image_layout "
          "width=%u height=%u encoding=%s",
          pair.left->width, pair.left->height, pair.left->encoding.c_str());
      return;
    }
    pending_.push_back(std::move(pair));
  }

  // The rotation the gyroscope saw between two frame stamps, as the turn of
  // a direction from the previous left camera frame into the current one.
  [[nodiscard]] Eigen::Matrix3d cameraTurn(const std::int64_t from_ns,
                                           const std::int64_t to_ns,
                                           const Eigen::Vector3d& bias) {
    Eigen::Matrix3d body_turn = Eigen::Matrix3d::Identity();
    std::int64_t previous_ns = from_ns;
    for (const GyroSample& sample : gyro_) {
      if (sample.stamp_ns <= from_ns) {
        continue;
      }
      const std::int64_t until_ns = std::min(sample.stamp_ns, to_ns);
      const Eigen::Vector3d turned =
          (sample.rate_radps - bias) *
          (1.0e-9 * static_cast<double>(until_ns - previous_ns));
      const double angle = turned.norm();
      if (angle > 1.0e-12) {
        body_turn = body_turn * Eigen::AngleAxisd{angle, turned / angle};
      }
      previous_ns = until_ns;
      if (sample.stamp_ns >= to_ns) {
        break;
      }
    }
    while (gyro_.size() > 1U && gyro_[1U].stamp_ns <= to_ns) {
      gyro_.pop_front();
    }
    return camera_to_body_.transpose() * body_turn.transpose() * camera_to_body_;
  }

  // A filter no feature has aided yet stands where the vehicle was declared
  // to stand: the vehicle has not left the ground, for the autopilot has no
  // position to take off on before the filter is aided. Started afresh
  // there, it never carries the drift of its IMU alone, nor starts again on
  // the autopilot's pose, which had nothing to vouch for it: on the dark
  // hall's pad, where the estimator found nothing to track for nine
  // minutes, its height ran to 380 m, a hole in the IMU stream restarted it
  // on the autopilot's pose 16 m off, and the vehicle took off on it and
  // struck a wall (r1466).
  void onGnss(const px4_msgs::msg::SensorGps& message) {
    const AutopilotGnssFix fix = px4SensorGpsToAutopilotGnssFix(message);
    const std::optional<std::int64_t> stamp_ns = time_mapper_.px4LocalToRosTimeNs(
        static_cast<std::int64_t>(fix.timestamp_us) * 1000);
    GnssFixSample sample =
        gnssFixSample(fix, stamp_ns.value_or(get_clock()->now().nanoseconds()));
    sample.arrival_ns = get_clock()->now().nanoseconds();
    const bool initialized = odometry_->initialized();
    std::optional<VisualInertialEstimate> predicted;
    if (initialized) {
      predicted = odometry_->predicted(sample.stamp_ns);
    }
    const GnssInputDecision decision =
        gnss_input_->fix(sample, initialized, predicted ? &*predicted : nullptr);
    if (!decision.reference_ready) {
      return;
    }
    if (decision.fuse) {
      odometry_->addAbsolutePosition(decision.position_ned_m, decision.sigma_m,
                                     decision.unaided_drift_m);
      ++gnss_fused_;
    }
    declareGnssTrust(get_logger(), *mission_event_pub_, decision.verdict,
                     decision.position_residual_m, gnss_trusted_declared_);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "%s",
                         GnssInput::summary(decision, fix, gnss_fused_).c_str());
  }

  void startAfresh(const char* reason) {
    odometry_ = std::make_unique<VisualInertialOdometry>(odometry_config_);
    imu_samples_at_start_ = imu_samples_;
    previous_frame_stamp_ns_ = 0;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                         "VISUAL_INERTIAL_ODOMETRY_START_AFRESH reason=%s", reason);
  }

  void onPair(const Pair& pair) {
    if (!odometry_->initialized()) {
      if (imu_samples_ - imu_samples_at_start_ < kRestSamplesBeforeInitialPose) {
        return;
      }
      odometry_->initialize(pair.stamp_ns, Eigen::Vector3d::Zero(),
                            initial_heading_ned_rad_);
      initialized_stamp_ns_ = pair.stamp_ns;
    }
    const auto started = std::chrono::steady_clock::now();
    // The tracker reads the buffers only: a header over the message's bytes,
    // not a copy (the relay publishes rows without padding, step == width).
    const auto grey = [](const sensor_msgs::msg::Image& image) {
      return cv::Mat{image.data, false}.reshape(1, static_cast<int>(image.height));
    };
    std::optional<Eigen::Matrix3d> turn;
    if (previous_frame_stamp_ns_ > 0) {
      turn = cameraTurn(previous_frame_stamp_ns_, pair.stamp_ns, gyro_bias_);
    }
    const std::vector<StereoFeatureObservation> observations = [&] {
      const std::array<cv::Mat, 2> rectified_pair =
          rectifier_->rectify(grey(*pair.left), grey(*pair.right));
      return tracker_->track(rectified_pair[0], rectified_pair[1], turn);
    }();
    VisualInertialEstimate estimate = odometry_->addFrame(pair.stamp_ns, observations);
    previous_frame_stamp_ns_ = pair.stamp_ns;
    aided_ = aided_ || estimate.healthy;
    // Time for its tracks to grow and aid it (a few frames), and no more.
    constexpr std::int64_t kNeverAidedRestartNs{5'000'000'000};
    if (!aided_ && pair.stamp_ns - initialized_stamp_ns_ > kNeverAidedRestartNs) {
      startAfresh("never_aided");
      return;
    }
    gyro_bias_ = estimate.gyro_bias_radps;
    const double frame_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - started)
                                .count();
    ++frames_;
    if (!estimate.healthy) {
      ++frames_without_estimate_;
    }
    // The pose in the map's frame, as the autopilot receives it.
    applyMapFrame(estimate, pair.stamp_ns);
    publishPose(estimate, pair.left->header.stamp);
    const Point2 map_xy = transform_.localPositionToMap(
        Point2{estimate.position_ned_m.x(), estimate.position_ned_m.y()});
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "VISUAL_INERTIAL_ODOMETRY healthy=%s void=%s tracked=%zu candidates=%zu "
        "used=%zu "
        "gated=%zu untriangulated=%zu residual_sigma=%.2f "
        "weakest_velocity_sigma_mps=%.3f "
        "clones=%zu speed_mps=%.2f frame_ms=%.1f imu_lag_ms=%.1f imu_gap_max_ms=%.1f "
        "pose_age_max_ms=%.1f "
        "imu_samples=%" PRIu64 " frames=%" PRIu64 " frames_without_estimate=%" PRIu64
        " unmapped_imu=%" PRIu64 " published_poses=%" PRIu64
        " position=(%.2f,%.2f,%.2f) yaw=%.3f velocity_sigma_ned=(%.3f,%.3f,%.3f)"
        " accelerometer_bias=(%.3f,%.3f,%.3f) accelerometer_bias_sigma=(%.3f,%.3f,%.3f)"
        " heading_sigma_deg=%.2f position_sigma_m=(%.2f,%.2f,%.2f)"
        " magnetic_innovation_deg=%.2f magnetic_age_ms=%.1f",
        estimate.healthy ? "true" : "false", estimate.in_void ? "true" : "false",
        estimate.tracked_features, estimate.candidate_features, estimate.used_features,
        estimate.gated_features, estimate.untriangulated_features,
        estimate.residual_rms_sigma, estimate.weakest_velocity_sigma_mps,
        estimate.clones, estimate.velocity_ned_mps.norm(), frame_ms,
        1.0e-6 * static_cast<double>(estimate.imu_lag_ns),
        1.0e-6 * static_cast<double>(imu_gap_max_ns_),
        1.0e-6 * static_cast<double>(pose_age_max_ns_), imu_samples_, frames_,
        frames_without_estimate_, unmapped_imu_samples_, published_poses_, map_xy.x,
        map_xy.y, -estimate.position_ned_m.z() + transform_.map_origin.z,
        mapYaw(estimate), std::sqrt(estimate.velocity_variance_m2ps2.x()),
        std::sqrt(estimate.velocity_variance_m2ps2.y()),
        std::sqrt(estimate.velocity_variance_m2ps2.z()),
        estimate.accelerometer_bias_mps2.x(), estimate.accelerometer_bias_mps2.y(),
        estimate.accelerometer_bias_mps2.z(),
        std::sqrt(estimate.accelerometer_bias_variance_m2ps4.x()),
        std::sqrt(estimate.accelerometer_bias_variance_m2ps4.y()),
        std::sqrt(estimate.accelerometer_bias_variance_m2ps4.z()),
        180.0 / M_PI * std::sqrt(estimate.orientation_variance_rad2.z()),
        std::sqrt(estimate.position_variance_m2.x()),
        std::sqrt(estimate.position_variance_m2.y()),
        std::sqrt(estimate.position_variance_m2.z()),
        180.0 / M_PI * estimate.magnetic_innovation_rad,
        1.0e-6 * static_cast<double>(estimate.magnetic_reading_age_ns));
    if (frames_ % 8U == 0U) {
      imu_gap_max_ns_ = 0;
      pose_age_max_ns_ = 0;
    }
  }

  // One depth frame against the estimator's long-lived map
  // (visual_inertial_odometry_node_map.cpp).
  void onDepth(const sensor_msgs::msg::PointCloud2& cloud);

  // Specification K28: a filter lost in a hole of its IMU stream (r720,
  // r723: a host frozen for seconds) stayed lost for the flight and the
  // autopilot landed the vehicle. Now the filter starts again on the
  // autopilot's own pose and velocity, which it carried through the hole on
  // its IMU: its frame stays the autopilot's, so the autopilot sees no reset
  // and the navigation stays open (r1294: a restart 3.4 m off reset the
  // autopilot by 3.7 m and the navigation closed for the flight). It starts
  // on the stream's first sample after the hole, healthy from it: the
  // autopilot holds its position valid for about 4.6 s without the external
  // odometry, and a restart that waited for the depth and for features came
  // 1.6 s after a 4 s hole, when the autopilot's failsafe had already let the
  // vehicle drift blind at 2 m/s into a wall (r1296, r1306). Its moment is
  // the autopilot's attitude's own sample on the IMU's clock, the position
  // carried from its own sample (r1304: started at an IMU sample a second
  // behind the attitude, the filter turned through that second twice).
  // Under the odometry mutex; returns whether the filter started.
  bool reinitialiseFromAutopilot() {
    // Both the position and the attitude this recent (they arrive at 50 Hz
    // and more).
    constexpr std::int64_t kAutopilotStateFreshNs{200'000'000};
    std::optional<AutopilotLocalState> local_state;
    std::optional<AutopilotAttitude> attitude;
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    {
      const std::scoped_lock lock{autopilot_state_mutex_};
      if (now_ns - autopilot_state_ros_ns_ <= kAutopilotStateFreshNs &&
          now_ns - autopilot_attitude_ros_ns_ <= kAutopilotStateFreshNs) {
        local_state = autopilot_local_state_;
        attitude = autopilot_attitude_;
      }
    }
    // The autopilot's position is taken whatever it calls its validity: it
    // stops vouching for it within a second or two of the external odometry
    // falling silent (r1283: invalid 0.8 s after a 2 s hole) while its
    // inertial dead reckoning is still the best prior there is.
    const auto finite3 = [](const auto& value) {
      return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    };
    if (!local_state.has_value() || !attitude.has_value() ||
        !finite3(local_state->position) || !finite3(local_state->velocity)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "VISUAL_INERTIAL_ODOMETRY_REINITIALISATION outcome=no_prior");
      return false;
    }
    const LidarPoseSourceStampResult position_stamp = resolveLidarPoseSourceStamp(
        time_mapper_, local_state->timestamp_sample_us, now_ns);
    const LidarPoseSourceStampResult attitude_stamp = resolveLidarPoseSourceStamp(
        time_mapper_, attitude->timestamp_sample_us, now_ns);
    if (!position_stamp.resolved() || !attitude_stamp.resolved()) {
      return false;
    }
    const std::int64_t restart_ns = attitude_stamp.mapped_ros_stamp_ns;
    // The autopilot's pose in the frame the filter publishes in: its position
    // in the map's frame back to the local NED, its velocity likewise, its
    // attitude's quaternion turning the body into NED (r1288: inverted, the
    // heading came out mirrored).
    const Point2 local_xy = transform_.mapPositionToLocal(
        Point2{local_state->position.x, local_state->position.y});
    const Point2 local_velocity = transform_.mapVectorToLocal(
        Point2{local_state->velocity.x, local_state->velocity.y});
    const Eigen::Vector3d velocity_ned{local_velocity.x, local_velocity.y,
                                       -local_state->velocity.z};
    const Eigen::Vector3d position_ned =
        Eigen::Vector3d{local_xy.x, local_xy.y,
                        -(local_state->position.z - transform_.map_origin.z)} +
        velocity_ned * (1.0e-9 * static_cast<double>(
                                     restart_ns - position_stamp.mapped_ros_stamp_ns));
    const Eigen::Quaterniond rotation_ned =
        Eigen::Quaterniond{attitude->quaternion[0], attitude->quaternion[1],
                           attitude->quaternion[2], attitude->quaternion[3]}
            .normalized();
    // The filter's own frame lies the applied map offset behind the published
    // one.
    const MapFrameOffset applied = map_registration_.applied();
    const Eigen::Matrix3d rotation = applied.rotation();
    odometry_->reinitialize(
        restart_ns, rotation.transpose() * (position_ned - applied.translation),
        rotation.transpose() * velocity_ned,
        Eigen::Quaterniond{rotation.transpose() * rotation_ned.toRotationMatrix()});
    ++reinitialisations_;
    const Eigen::Matrix3d body_to_ned = rotation_ned.toRotationMatrix();
    RCLCPP_WARN(get_logger(),
                "VISUAL_INERTIAL_ODOMETRY_REINITIALISATION outcome=started "
                "position=(%.2f,%.2f,%.2f) heading_deg=%.1f attitude_age_ms=%.0f "
                "reinitialisations=%" PRIu64,
                position_ned.x(), position_ned.y(), position_ned.z(),
                std::atan2(body_to_ned(1, 0), body_to_ned(0, 0)) * 180.0 / M_PI,
                1.0e-6 * static_cast<double>(now_ns - restart_ns), reinitialisations_);
    return true;
  }

  // The estimate in the map's frame, as published (the module's applied offset).
  void applyMapFrame(VisualInertialEstimate& estimate, const std::int64_t stamp_ns) {
    map_registration_.applyFrame(estimate, stamp_ns);
  }

  [[nodiscard]] double mapYaw(const VisualInertialEstimate& estimate) const noexcept {
    const Eigen::Quaterniond& q = estimate.body_to_ned;
    const double heading = std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    return transform_.px4HeadingToMapYaw(heading);
  }

  // The estimate in the map frame, yaw only in the orientation: what the
  // mission check compares with the true pose. The health rides in the frame
  // name, as the lidar-inertial estimate's does.
  void publishPose(const VisualInertialEstimate& estimate,
                   const builtin_interfaces::msg::Time& stamp) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = estimate.healthy ? "map" : "map_unhealthy";
    const Point2 map_xy = transform_.localPositionToMap(
        Point2{estimate.position_ned_m.x(), estimate.position_ned_m.y()});
    pose.pose.position.x = map_xy.x;
    pose.pose.position.y = map_xy.y;
    pose.pose.position.z = -estimate.position_ned_m.z() + transform_.map_origin.z;
    const double yaw = mapYaw(estimate);
    pose.pose.orientation.w = std::cos(0.5 * yaw);
    pose.pose.orientation.z = std::sin(0.5 * yaw);
    pose_pub_->publish(pose);
  }

  // The filter is read and advanced by the IMU and frame thread and by the
  // depth thread.
  std::mutex odometry_mutex_;
  std::unique_ptr<VisualInertialOdometry> odometry_;
  rclcpp::CallbackGroup::SharedPtr depth_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr depth_sub_;
  // The long-lived map, its registrations and the offset to the map's frame
  // (roadmap items 19 and 40); `map_registration` "measure" reads without moving.
  StereoMapRegistration map_registration_;
  bool map_registration_act_{true};
  Eigen::Vector3d left_camera_body_{Eigen::Vector3d::Zero()};
  std::unique_ptr<StereoFeatureTracker> tracker_;
  std::unique_ptr<AutopilotStateSource> autopilot_;
  // The autopilot's own estimate, the prior of an initialisation in flight.
  std::mutex autopilot_state_mutex_;
  std::optional<AutopilotLocalState> autopilot_local_state_;
  std::optional<AutopilotAttitude> autopilot_attitude_;
  std::int64_t autopilot_state_ros_ns_{0};
  std::int64_t autopilot_attitude_ros_ns_{0};
  std::int64_t maximum_unaided_ns_{0};
  bool restart_due_{false};
  // Whether any frame has aided the filter since the node started (onPair).
  bool aided_{false};
  VisualInertialOdometryConfig odometry_config_;
  std::uint64_t imu_samples_at_start_{0U};
  std::int64_t initialized_stamp_ns_{0};
  std::unique_ptr<AutopilotStateSource> autopilot_pose_;
  rclcpp::CallbackGroup::SharedPtr autopilot_pose_group_;
  std::uint64_t reinitialisations_{0U};
  // The first try of the restart under way, on the node's clock; zero: none.
  rclcpp::CallbackGroup::SharedPtr group_;
  std::unique_ptr<StepAcknowledger> step_acknowledger_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_event_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_pub_;
  bool publish_to_autopilot_{false};
  std::int64_t last_autopilot_stamp_ns_{0};
  bool dead_reckoning_declared_{false};
  // Air at 1.225 kg/m^3 under 9.807 m/s^2: 12.0 Pa a metre near the ground.
  static constexpr double kPressurePerMetrePa{12.013};
  static constexpr double kPressureSmoothing{0.04};
  double smoothed_pressure_pa_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr barometer_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_sub_;
  std::int64_t magnetometer_logged_ns_{0};
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr void_range_sub_;
  bool void_declared_{false};
  rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr gnss_sub_;
  std::unique_ptr<GnssInput> gnss_input_;
  bool gnss_trusted_declared_{false};
  std::uint64_t gnss_fused_{0U};
  std::uint64_t published_poses_{0U};
  sensor_msgs::msg::Image::ConstSharedPtr left_;
  sensor_msgs::msg::Image::ConstSharedPtr right_;
  std::deque<Pair> pending_;
  std::deque<GyroSample> gyro_;
  Px4MapFrameTransform transform_;
  Px4RosTimeMapper time_mapper_;
  // The rectified left camera's optical frame in the body (the features'),
  // and the physical one's (the depth's).
  Eigen::Matrix3d camera_to_body_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d left_camera_to_body_{Eigen::Matrix3d::Identity()};
  std::unique_ptr<StereoRectifier> rectifier_;
  Eigen::Vector3d gyro_bias_{Eigen::Vector3d::Zero()};
  std::size_t image_width_{0U};
  std::size_t image_height_{0U};
  double initial_heading_ned_rad_{0.0};
  std::int64_t frame_stamp_offset_ns_{0};
  std::int64_t previous_frame_stamp_ns_{0};
  std::int64_t last_imu_stamp_ns_{0};
  // The longest interval between consecutive IMU samples over the last eight
  // frames: past 50 ms the filter treats it as a hole.
  std::int64_t imu_gap_max_ns_{0};
  std::int64_t pose_age_max_ns_{0};
  std::uint64_t imu_samples_{0U};
  std::uint64_t unmapped_imu_samples_{0U};
  std::uint64_t frames_{0U};
  std::uint64_t frames_without_estimate_{0U};
};

} // namespace drone_city_nav::visual_inertial_odometry_node_internal
