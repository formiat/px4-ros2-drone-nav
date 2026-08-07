#include "drone_city_nav/radar_target_tracker_node.hpp"

#include "drone_city_nav/msg/radar_scan.hpp"
#include "drone_city_nav/msg/target_track.hpp"
#include "drone_city_nav/msg/vehicle_navigation_state.hpp"
#include "drone_city_nav/radar_target_tracker.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cinttypes>
#include <cstdint>
#include <memory>
#include <optional>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>
#include <string>

#include "intercept_ros_utils.hpp"

namespace drone_city_nav {

class RadarTargetTrackerNode final : public rclcpp::Node {
public:
  explicit RadarTargetTrackerNode(const rclcpp::NodeOptions& options)
      : Node{"radar_target_tracker_node", options},
        ownship_history_{RadarOwnshipHistoryConfig{
            .retention_ns = static_cast<std::int64_t>(
                declare_parameter<double>("ownship_history_retention_s", 5.0) * 1.0e9),
            .maximum_extrapolation_ns = static_cast<std::int64_t>(
                declare_parameter<double>("ownship_maximum_extrapolation_s", 0.1) *
                1.0e9),
        }},
        tracker_{RadarTargetTrackerConfig{
            .maximum_update_interval_s =
                declare_parameter<double>("maximum_update_interval_s", 4.0),
            .position_correction_gain =
                declare_parameter<double>("position_correction_gain", 1.0),
            .velocity_correction_gain =
                declare_parameter<double>("velocity_correction_gain", 0.5),
            .high_rate_velocity_correction_gain =
                declare_parameter<double>("high_rate_velocity_correction_gain", 1.0),
            .maximum_ownship_stamp_error_s =
                declare_parameter<double>("maximum_ownship_stamp_error_s", 0.05),
            .track_id = static_cast<std::uint64_t>(
                declare_parameter<std::int64_t>("track_id", 1)),
        }} {
    expected_radar_frame_ =
        declare_parameter<std::string>("expected_radar_frame", "radar_yaw");
    output_frame_ = declare_parameter<std::string>("output_frame", "map");
    const auto state_qos = rclcpp::QoS{10}.best_effort();
    ownship_state_sub_ = create_subscription<msg::VehicleNavigationState>(
        declare_parameter<std::string>("ownship_state_topic",
                                       "/vehicles/interceptor/state"),
        state_qos, [this](const msg::VehicleNavigationState::SharedPtr state) {
          if (ownship_history_.add(detail::vehicleState(*state))) {
            processPendingScan();
          }
        });
    radar_scan_sub_ = create_subscription<msg::RadarScan>(
        declare_parameter<std::string>("radar_scan_topic",
                                       "/vehicles/interceptor/radar/scan"),
        rclcpp::QoS{10}.reliable(),
        [this](const msg::RadarScan::SharedPtr scan) { onRadarScan(*scan); });
    track_pub_ = create_publisher<msg::TargetTrack>(
        declare_parameter<std::string>("target_track_topic",
                                       "/vehicles/interceptor/target_track"),
        rclcpp::QoS{1}.reliable().transient_local());
    track_readiness_pub_ = create_publisher<std_msgs::msg::Bool>(
        declare_parameter<std::string>("target_track_readiness_topic",
                                       "/vehicles/interceptor/target_track_ready"),
        rclcpp::QoS{1}.reliable().transient_local());
    publishTrackReadiness(false);
    RCLCPP_INFO(get_logger(),
                "Radar target tracker ready: input_frame='%s' output_frame='%s'",
                expected_radar_frame_.c_str(), output_frame_.c_str());
  }

private:
  void publishTrackReadiness(const bool ready) {
    std_msgs::msg::Bool message;
    message.data = ready;
    track_readiness_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "RADAR_TRACK_READY ready=%s", ready ? "true" : "false");
  }

  void onRadarScan(const msg::RadarScan& scan) {
    if (scan.header.frame_id != expected_radar_frame_ || scan.detections.empty() ||
        scan.cadence_mode > msg::RadarScan::CADENCE_MODE_TRACK) {
      RCLCPP_WARN(get_logger(),
                  "RADAR_TRACK rejected=true reason=invalid_scan frame='%s' "
                  "detections=%zu",
                  scan.header.frame_id.c_str(), scan.detections.size());
      return;
    }
    if (scan.scan_sequence <= last_scan_sequence_) {
      return;
    }
    pending_scan_ = scan;
    processPendingScan();
  }

  void processPendingScan() {
    if (!pending_scan_.has_value()) {
      return;
    }
    const std::int64_t stamp_ns = detail::timeNanoseconds(pending_scan_->header.stamp);
    const std::optional<TimedVehicleState> ownship = ownship_history_.sample(stamp_ns);
    if (!ownship.has_value()) {
      return;
    }
    const msg::RadarDetection& input = pending_scan_->detections.front();
    const RadarDetectionSample detection{
        .detection_id = input.detection_id,
        .range_m = input.range_m,
        .azimuth_rad = input.azimuth_rad,
        .elevation_rad = input.elevation_rad,
        .radial_velocity_mps = input.radial_velocity_mps,
    };
    const RadarTrackEstimate estimate = tracker_.update(
        *ownship, detection, stamp_ns, pending_scan_->scan_sequence,
        pending_scan_->cadence_mode == msg::RadarScan::CADENCE_MODE_TRACK
            ? RadarTrackerUpdateMode::kTrack
            : RadarTrackerUpdateMode::kSearch);
    if (!estimate.position_valid) {
      RCLCPP_WARN(get_logger(),
                  "RADAR_TRACK rejected=true reason=tracker_update_failed "
                  "scan_sequence=%" PRIu64,
                  pending_scan_->scan_sequence);
      pending_scan_.reset();
      return;
    }

    msg::TargetTrack track;
    track.header.stamp = pending_scan_->header.stamp;
    track.header.frame_id = output_frame_;
    track.track_id = estimate.track_id;
    track.source_scan_sequence = estimate.source_scan_sequence;
    track.source_detection_id = estimate.source_detection_id;
    track.position.x = estimate.position.x;
    track.position.y = estimate.position.y;
    track.position.z = estimate.position.z;
    track.velocity.x = estimate.velocity.x;
    track.velocity.y = estimate.velocity.y;
    track.velocity.z = estimate.velocity.z;
    track.position_valid = estimate.position_valid;
    track.velocity_valid = estimate.velocity_valid;
    track.status = estimate.velocity_valid ? msg::TargetTrack::STATUS_TRACKING
                                           : msg::TargetTrack::STATUS_INITIALIZING;
    track_pub_->publish(track);
    if (!track_ready_) {
      track_ready_ = true;
      publishTrackReadiness(true);
    }
    last_scan_sequence_ = pending_scan_->scan_sequence;
    RCLCPP_INFO(get_logger(),
                "RADAR_TRACK status=%s track_id=%" PRIu64 " scan_sequence=%" PRIu64
                " measurement_count=%zu velocity_valid=%s cadence_mode=%s "
                "cadence_reason=%u velocity_correction_gain=%.3f",
                estimate.velocity_valid ? "tracking" : "initializing",
                estimate.track_id, estimate.source_scan_sequence,
                estimate.measurement_count, estimate.velocity_valid ? "true" : "false",
                pending_scan_->cadence_mode == msg::RadarScan::CADENCE_MODE_TRACK
                    ? "track"
                    : "search",
                static_cast<unsigned int>(pending_scan_->cadence_reason),
                estimate.velocity_correction_gain);
    pending_scan_.reset();
  }

  RadarOwnshipHistory ownship_history_;
  RadarTargetTracker tracker_;
  std::optional<msg::RadarScan> pending_scan_;
  std::string expected_radar_frame_;
  std::string output_frame_;
  std::uint64_t last_scan_sequence_{0U};
  bool track_ready_{false};
  rclcpp::Subscription<msg::VehicleNavigationState>::SharedPtr ownship_state_sub_;
  rclcpp::Subscription<msg::RadarScan>::SharedPtr radar_scan_sub_;
  rclcpp::Publisher<msg::TargetTrack>::SharedPtr track_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr track_readiness_pub_;
};

std::shared_ptr<rclcpp::Node>
makeRadarTargetTrackerNode(const rclcpp::NodeOptions& options) {
  return std::make_shared<RadarTargetTrackerNode>(options);
}

} // namespace drone_city_nav

RCLCPP_COMPONENTS_REGISTER_NODE(drone_city_nav::RadarTargetTrackerNode)
