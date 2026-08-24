#include "drone_city_nav/mission_waypoint_acknowledgement_admission.hpp"
#include "drone_city_nav/mission_waypoint_sequence.hpp"
#include "drone_city_nav/msg/mission_waypoint_acknowledgement.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/types.hpp"

#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <bit>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) {
    return 0;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

[[nodiscard]] bool finitePoint(const geometry_msgs::msg::Point& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

[[nodiscard]] Point3 point3(const geometry_msgs::msg::Point& point) noexcept {
  return Point3{point.x, point.y, point.z};
}

constexpr std::uint64_t kAcknowledgementFingerprintOffset{
    14'695'981'039'346'656'037ULL};
constexpr std::uint64_t kAcknowledgementFingerprintPrime{1'099'511'628'211ULL};

void appendAcknowledgementFingerprint(std::uint64_t& fingerprint,
                                      const std::uint64_t word) noexcept {
  for (std::size_t byte_index = 0U; byte_index < sizeof(word); ++byte_index) {
    fingerprint ^= (word >> (byte_index * 8U)) & 0xFFU;
    fingerprint *= kAcknowledgementFingerprintPrime;
  }
}

[[nodiscard]] std::uint64_t canonicalDoubleBits(const double value) noexcept {
  if (value == 0.0) {
    return 0U;
  }
  if (std::isnan(value)) {
    return 0x7FF8'0000'0000'0000ULL;
  }
  return std::bit_cast<std::uint64_t>(value);
}

[[nodiscard]] std::uint64_t acknowledgementContentFingerprint(
    const msg::MissionWaypointAcknowledgement& acknowledgement) noexcept {
  std::uint64_t fingerprint{kAcknowledgementFingerprintOffset};
  appendAcknowledgementFingerprint(fingerprint, 1U); // Fingerprint schema.
  appendAcknowledgementFingerprint(
      fingerprint, static_cast<std::uint64_t>(acknowledgement.header.stamp.sec));
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.header.stamp.nanosec);
  for (const char character : acknowledgement.header.frame_id) {
    appendAcknowledgementFingerprint(fingerprint,
                                     static_cast<unsigned char>(character));
  }
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.header.frame_id.size());
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.producer_instance_id);
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.acknowledgement_sequence);
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.mission_epoch);
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.completed_waypoint_index);
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.completed_waypoint_count);
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.waypoint_count);
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.active_waypoint_index);
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.mission_completed ? 1U : 0U);
  for (const geometry_msgs::msg::Point* const point :
       {&acknowledgement.completed_goal, &acknowledgement.route_target,
        &acknowledgement.stationary_hold_position}) {
    appendAcknowledgementFingerprint(fingerprint, canonicalDoubleBits(point->x));
    appendAcknowledgementFingerprint(fingerprint, canonicalDoubleBits(point->y));
    appendAcknowledgementFingerprint(fingerprint, canonicalDoubleBits(point->z));
  }
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.horizon_producer_instance_id);
  appendAcknowledgementFingerprint(fingerprint, acknowledgement.horizon_sequence);
  appendAcknowledgementFingerprint(fingerprint,
                                   acknowledgement.offboard_producer_instance_id);
  for (const builtin_interfaces::msg::Time* const stamp :
       {&acknowledgement.horizon_valid_from, &acknowledgement.horizon_valid_until,
        &acknowledgement.witness_stamp}) {
    appendAcknowledgementFingerprint(fingerprint,
                                     static_cast<std::uint64_t>(stamp->sec));
    appendAcknowledgementFingerprint(fingerprint, stamp->nanosec);
  }
  return fingerprint == 0U ? 1U : fingerprint;
}

[[nodiscard]] const char* acknowledgementRejectionReason(
    const MissionWaypointAcknowledgementAdmissionResult& admission) noexcept {
  if (admission.conflict) {
    return "identity_conflict";
  }
  if (admission.replay) {
    return "replay";
  }
  if (admission.aggregate_replay) {
    return "aggregate_replay";
  }
  if (admission.stale) {
    return "stale";
  }
  if (admission.retired_capacity_exhausted) {
    return "retired_capacity_exhausted";
  }
  if (admission.prospective_capacity_exhausted) {
    return "prospective_capacity_exhausted";
  }
  return "invalid";
}

} // namespace

class MissionMonitorNode final : public rclcpp::Node {
public:
  MissionMonitorNode()
      : Node{"mission_monitor_node"} {
    start_ = Point2{declare_parameter<double>("start_x_m", 54.0),
                    declare_parameter<double>("start_y_m", 54.0)};
    px4_local_origin_ = Point2{declare_parameter<double>("px4_local_origin_x_m", 54.0),
                               declare_parameter<double>("px4_local_origin_y_m", 54.0)};
    spawn_tolerance_m_ = declare_parameter<double>("spawn_tolerance_m", 1.0);
    minimum_movement_m_ = declare_parameter<double>("min_movement_distance_m", 5.0);
    acknowledgement_target_tolerance_m_ =
        declare_parameter<double>("mission_waypoint_target_match_tolerance_m", 1.0e-3);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    if (!std::isfinite(acknowledgement_target_tolerance_m_) ||
        acknowledgement_target_tolerance_m_ < 0.0 || frame_id_.empty()) {
      throw std::invalid_argument{
          "mission acknowledgement frame and tolerance must be valid"};
    }
    shutdown_on_result_ = declare_parameter<bool>("shutdown_on_result", false);
    waypoints_ =
        missionWaypointsFromFlatParameters(declare_parameter<std::vector<double>>(
            "mission_goal_sequence_xyz_m", std::vector<double>{}));
    goal_ = Point2{waypoints_.front().x, waypoints_.front().y};

    const auto px4_qos =
        rclcpp::QoS{rclcpp::KeepLast{10}}.best_effort().durability_volatile();
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        declare_parameter<std::string>("px4_local_position_topic",
                                       "/fmu/out/vehicle_local_position"),
        px4_qos, [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr message) {
          onLocalPosition(*message);
        });
    vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        declare_parameter<std::string>("px4_vehicle_status_topic",
                                       "/fmu/out/vehicle_status"),
        px4_qos, [this](const px4_msgs::msg::VehicleStatus::SharedPtr message) {
          armed_seen_ =
              armed_seen_ ||
              message->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        });
    vehicle_destroyed_sub_ = create_subscription<msg::VehicleDestroyed>(
        declare_parameter<std::string>("vehicle_destroyed_topic",
                                       "/drone_city_nav/vehicle_destroyed"),
        rclcpp::QoS{rclcpp::KeepLast{1}}.reliable().transient_local(),
        [this](const msg::VehicleDestroyed::SharedPtr message) {
          if (!result_reported_) {
            RCLCPP_ERROR(get_logger(),
                         "MISSION_CHECK vehicle_destroyed=true role=%u cause=%u "
                         "drone_collision='%s' obstacle_collision='%s' "
                         "event_position=(%.3f, %.3f, %.3f)",
                         static_cast<unsigned>(message->vehicle_role),
                         static_cast<unsigned>(message->death_cause),
                         message->drone_collision.c_str(),
                         message->obstacle_collision.c_str(), message->event_position.x,
                         message->event_position.y, message->event_position.z);
            report(false, "vehicle_destroyed");
          }
        });
    waypoint_acknowledgement_sub_ =
        create_subscription<msg::MissionWaypointAcknowledgement>(
            declare_parameter<std::string>(
                "mission_waypoint_acknowledgement_topic",
                "/drone_city_nav/mission_waypoint_acknowledgement"),
            rclcpp::QoS{32}.reliable().transient_local(),
            [this](const msg::MissionWaypointAcknowledgement::SharedPtr message) {
              onWaypointAcknowledgement(*message);
            });
    summary_timer_ =
        create_wall_timer(std::chrono::seconds{5}, [this] { logSummary(); });
    RCLCPP_INFO(get_logger(),
                "Mission monitor ready: start=(%.2f, %.2f) waypoint_count=%zu "
                "first_goal=(%.2f, %.2f)",
                start_.x, start_.y, waypoints_.size(), goal_.x, goal_.y);
  }

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& message) {
    if (result_reported_ || !message.xy_valid || !message.z_valid ||
        !message.v_xy_valid || !message.v_z_valid || !std::isfinite(message.x) ||
        !std::isfinite(message.y) || !std::isfinite(message.z) ||
        !std::isfinite(message.vx) || !std::isfinite(message.vy) ||
        !std::isfinite(message.vz)) {
      latest_position_valid_ = false;
      return;
    }
    latest_position_ = Point2{static_cast<double>(message.x) + px4_local_origin_.x,
                              static_cast<double>(message.y) + px4_local_origin_.y};
    latest_altitude_m_ = -static_cast<double>(message.z);
    latest_speed_mps_ = std::hypot(
        std::hypot(static_cast<double>(message.vx), static_cast<double>(message.vy)),
        static_cast<double>(message.vz));
    latest_position_valid_ = true;
    const double start_distance_m = distance(latest_position_, start_);
    const double goal_distance_m = distance(latest_position_, goal_);
    if (!first_position_seen_) {
      first_position_seen_ = true;
      spawn_distance_m_ = start_distance_m;
      spawn_ok_ = spawn_distance_m_ <= spawn_tolerance_m_;
    }
    maximum_distance_from_start_m_ =
        std::max(maximum_distance_from_start_m_, start_distance_m);
    minimum_goal_distance_m_ = std::min(minimum_goal_distance_m_, goal_distance_m);
    maximum_speed_mps_ = std::max(maximum_speed_mps_, latest_speed_mps_);
    speed_sum_mps_ += latest_speed_mps_;
    ++speed_samples_;
    moved_ = moved_ || maximum_distance_from_start_m_ >= minimum_movement_m_;
  }

  void onWaypointAcknowledgement(
      const msg::MissionWaypointAcknowledgement& acknowledgement) {
    if (result_reported_) {
      return;
    }
    const std::int64_t receive_stamp_ns = now().nanoseconds();
    const std::int64_t source_stamp_ns = timeNanoseconds(acknowledgement.header.stamp);
    const Point3 completed_goal = point3(acknowledgement.completed_goal);
    const Point3 route_target = point3(acknowledgement.route_target);
    const Point3 hold_position = point3(acknowledgement.stationary_hold_position);
    const bool waypoint_index_valid =
        acknowledgement.completed_waypoint_index < waypoints_.size();
    const Point3 expected_goal =
        waypoint_index_valid ? waypoints_[acknowledgement.completed_waypoint_index]
                             : Point3{};
    const bool payload_valid =
        acknowledgement.header.frame_id == frame_id_ &&
        finitePoint(acknowledgement.completed_goal) &&
        finitePoint(acknowledgement.route_target) &&
        finitePoint(acknowledgement.stationary_hold_position) &&
        acknowledgement.waypoint_count == waypoints_.size() && waypoint_index_valid &&
        distance3D(completed_goal, expected_goal) <=
            acknowledgement_target_tolerance_m_ &&
        distance3D(route_target, expected_goal) <=
            acknowledgement_target_tolerance_m_ &&
        distance3D(hold_position, expected_goal) <= acknowledgement_target_tolerance_m_;
    const MissionWaypointAcknowledgementCandidate candidate{
        .producer_instance_id = acknowledgement.producer_instance_id,
        .acknowledgement_sequence = acknowledgement.acknowledgement_sequence,
        .content_fingerprint = acknowledgementContentFingerprint(acknowledgement),
        .mission_epoch = acknowledgement.mission_epoch,
        .horizon_producer_instance_id = acknowledgement.horizon_producer_instance_id,
        .horizon_sequence = acknowledgement.horizon_sequence,
        .offboard_producer_instance_id = acknowledgement.offboard_producer_instance_id,
        .source_stamp_ns = source_stamp_ns,
        .receive_stamp_ns = receive_stamp_ns,
        .horizon_valid_from_ns = timeNanoseconds(acknowledgement.horizon_valid_from),
        .horizon_valid_until_ns = timeNanoseconds(acknowledgement.horizon_valid_until),
        .witness_stamp_ns = timeNanoseconds(acknowledgement.witness_stamp),
        .completed_waypoint_index = acknowledgement.completed_waypoint_index,
        .completed_waypoint_count = acknowledgement.completed_waypoint_count,
        .waypoint_count = acknowledgement.waypoint_count,
        .active_waypoint_index = acknowledgement.active_waypoint_index,
        .completed_goal = completed_goal,
        .route_target = route_target,
        .stationary_hold_position = hold_position,
        .mission_completed = acknowledgement.mission_completed,
        .payload_valid = payload_valid,
    };
    const MissionWaypointAcknowledgementAdmissionResult admission =
        admitMissionWaypointAcknowledgement(waypoint_acknowledgement_admission_,
                                            candidate);
    if (admission.state_advanced) {
      waypoint_acknowledgement_admission_ = admission.next_state;
    }
    if (!admission.accept) {
      if (!admission.replay && !admission.aggregate_replay) {
        RCLCPP_WARN(get_logger(),
                    "MISSION_WAYPOINT_ACK_REJECTED producer=%" PRIu64
                    " acknowledgement=%" PRIu64 " reason=%s",
                    acknowledgement.producer_instance_id,
                    acknowledgement.acknowledgement_sequence,
                    acknowledgementRejectionReason(admission));
      }
      return;
    }

    completed_waypoint_count_ = acknowledgement.completed_waypoint_count;
    active_waypoint_index_ = acknowledgement.active_waypoint_index;
    if (!acknowledgement.mission_completed) {
      goal_ = Point2{waypoints_[active_waypoint_index_].x,
                     waypoints_[active_waypoint_index_].y};
      minimum_goal_distance_m_ = std::numeric_limits<double>::infinity();
      RCLCPP_INFO(
          get_logger(),
          "MISSION_WAYPOINT_REACHED completed_index=%u waypoint_count=%zu "
          "planner=%" PRIu64 " acknowledgement=%" PRIu64 " horizon=%" PRIu64
          " offboard=%" PRIu64 " next_goal=(%.2f,%.2f,%.2f)",
          acknowledgement.completed_waypoint_index, waypoints_.size(),
          acknowledgement.producer_instance_id,
          acknowledgement.acknowledgement_sequence, acknowledgement.horizon_sequence,
          acknowledgement.offboard_producer_instance_id,
          waypoints_[active_waypoint_index_].x, waypoints_[active_waypoint_index_].y,
          waypoints_[active_waypoint_index_].z);
      return;
    }
    RCLCPP_INFO(get_logger(),
                "MISSION_WAYPOINT_REACHED completed_index=%u waypoint_count=%zu "
                "planner=%" PRIu64 " acknowledgement=%" PRIu64 " terminal=true",
                acknowledgement.completed_waypoint_index, waypoints_.size(),
                acknowledgement.producer_instance_id,
                acknowledgement.acknowledgement_sequence);
    report(spawn_ok_ && moved_, spawn_ok_ && moved_ ? "none" : "mission_contract");
  }

  [[nodiscard]] double meanSpeed() const noexcept {
    return speed_samples_ == 0U ? std::numeric_limits<double>::quiet_NaN()
                                : speed_sum_mps_ / static_cast<double>(speed_samples_);
  }

  void report(const bool success, const std::string& reason) {
    result_reported_ = true;
    const char* format =
        "MISSION_RESULT success=%s reason='%s' spawn_distance=%.2f "
        "max_distance_from_start=%.2f min_goal_distance=%.2f waypoint_count=%zu "
        "completed_waypoints=%zu final_position=(%.2f, %.2f) final_altitude=%.2f "
        "final_speed=%.2f max_observed_speed=%.2f mean_observed_speed=%.2f";
    if (success) {
      RCLCPP_INFO(get_logger(), format, "true", reason.c_str(), spawn_distance_m_,
                  maximum_distance_from_start_m_, minimum_goal_distance_m_,
                  waypoints_.size(), completed_waypoint_count_, latest_position_.x,
                  latest_position_.y, latest_altitude_m_, latest_speed_mps_,
                  maximum_speed_mps_, meanSpeed());
    } else {
      RCLCPP_ERROR(get_logger(), format, "false", reason.c_str(), spawn_distance_m_,
                   maximum_distance_from_start_m_, minimum_goal_distance_m_,
                   waypoints_.size(), completed_waypoint_count_, latest_position_.x,
                   latest_position_.y, latest_altitude_m_, latest_speed_mps_,
                   maximum_speed_mps_, meanSpeed());
    }
    if (shutdown_on_result_) {
      shutdown_timer_ = create_wall_timer(std::chrono::milliseconds{100}, [this] {
        shutdown_timer_->cancel();
        rclcpp::shutdown();
      });
    }
  }

  void logSummary() {
    if (!latest_position_valid_ || result_reported_) {
      return;
    }
    RCLCPP_INFO(get_logger(),
                "Mission summary: spawn_ok=%s moved=%s armed_seen=%s "
                "position=(%.2f, %.2f) altitude=%.2f speed=%.2f "
                "active_waypoint=%zu/%zu distance_to_goal=%.2f "
                "min_goal_distance=%.2f",
                spawn_ok_ ? "true" : "false", moved_ ? "true" : "false",
                armed_seen_ ? "true" : "false", latest_position_.x, latest_position_.y,
                latest_altitude_m_, latest_speed_mps_, active_waypoint_index_ + 1U,
                waypoints_.size(), distance(latest_position_, goal_),
                minimum_goal_distance_m_);
  }

  Point2 start_{};
  Point2 goal_{};
  Point2 px4_local_origin_{};
  Point2 latest_position_{};
  std::vector<Point3> waypoints_;
  std::string frame_id_{"map"};
  double spawn_tolerance_m_{1.0};
  double minimum_movement_m_{5.0};
  double acknowledgement_target_tolerance_m_{1.0e-3};
  double latest_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double latest_speed_mps_{std::numeric_limits<double>::infinity()};
  double spawn_distance_m_{std::numeric_limits<double>::infinity()};
  double maximum_distance_from_start_m_{0.0};
  double minimum_goal_distance_m_{std::numeric_limits<double>::infinity()};
  double maximum_speed_mps_{0.0};
  double speed_sum_mps_{0.0};
  std::size_t speed_samples_{0U};
  std::size_t active_waypoint_index_{0U};
  std::size_t completed_waypoint_count_{0U};
  bool first_position_seen_{false};
  bool latest_position_valid_{false};
  bool spawn_ok_{false};
  bool moved_{false};
  bool armed_seen_{false};
  bool result_reported_{false};
  bool shutdown_on_result_{false};
  MissionWaypointAcknowledgementAdmissionState waypoint_acknowledgement_admission_{};
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr vehicle_destroyed_sub_;
  rclcpp::Subscription<msg::MissionWaypointAcknowledgement>::SharedPtr
      waypoint_acknowledgement_sub_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

} // namespace drone_city_nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<drone_city_nav::MissionMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
