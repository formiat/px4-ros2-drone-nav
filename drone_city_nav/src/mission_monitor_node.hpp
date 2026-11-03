#pragma once

#include "drone_city_nav/autopilot_state.hpp"
#include "drone_city_nav/autopilot_state_source.hpp"
#include "drone_city_nav/carried_light_judgment.hpp"
#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/footprint_geometry_3d.hpp"
#include "drone_city_nav/goal_reachability_proof_3d.hpp"
#include "drone_city_nav/main_battery_estimate.hpp"
#include "drone_city_nav/mission_waypoint_acknowledgement_admission.hpp"
#include "drone_city_nav/mission_waypoint_sequence.hpp"
#include "drone_city_nav/msg/mission_waypoint_acknowledgement.hpp"
#include "drone_city_nav/msg/navigation_health.hpp"
#include "drone_city_nav/msg/navigation_objective.hpp"
#include "drone_city_nav/msg/navigation_progress.hpp"
#include "drone_city_nav/msg/raw_obstacle_delta3_d.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot3_d.hpp"
#include "drone_city_nav/msg/vehicle_destroyed.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/operator_command.hpp"
#include "drone_city_nav/px4_map_frame_transform.hpp"
#include "drone_city_nav/return_trail.hpp"
#include "drone_city_nav/step_acknowledger.hpp"
#include "drone_city_nav/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <bit>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace drone_city_nav {

class MissionMonitorNode final : public rclcpp::Node {
public:
  MissionMonitorNode();

private:
  void onNavigationHealth(const msg::NavigationHealth& health);

  void onLocalState(const AutopilotLocalState& message);

  void
  onWaypointAcknowledgement(const msg::MissionWaypointAcknowledgement& acknowledgement);

  void onMemorySnapshot(const msg::RawObstacleSnapshot3D& snapshot);

  void onMemoryDelta(const msg::RawObstacleDelta3D& delta);

  static void
  applyMemoryChunks(ObservedOccupancyGrid3D& grid,
                    const std::vector<msg::ObservedObstacleChunk3D>& chunks) {
    for (const msg::ObservedObstacleChunk3D& message : chunks) {
      ObservedOccupancyGrid3D::Chunk chunk;
      chunk.observed = message.observed_words;
      chunk.occupied = message.occupied_words;
      static_cast<void>(grid.replaceChunk(
          OccupancyChunkIndex3D{message.x, message.y, message.z}, chunk));
    }
  }

  // After a return the proof runs to the start: a start proven unreachable as
  // well is logged, and the vehicle holds where it is, because the planner
  // has no route; a proof that opens again (the memory's closures decay) is
  // logged too, and the planner flies on by itself.
  void judgeReturnHome();

  // The way back is known and observed: the path flown so far at the
  // flight's mean speed, floored at the start's 0.5 m/s, errs on the long
  // side; the planner routes anew through the observed space, and over
  // twelve returns of item 19 the way back took 0.72 to 1.65 times the flight
  // out, so it is doubled, with a 20 s reserve. Logged beside the battery's
  // charge when a goal is given up.
  [[nodiscard]] double homeEstimateS() const noexcept;

  // The way home against the way out, which it retraces: 1.5 (the owner,
  // 2026-10-09; at 2.0 the low-battery flight came home with 90 of its 240 s
  // unspent, the way home having taken 62 s for 60 s out, r1185).
  static constexpr double kReturnEstimateMargin{1.5};
  static constexpr double kReturnReserveS{20.0};
  // The charge kept over what the way home needs when the vehicle turns
  // (specification K11, the owner's rule of 2026-10-05).
  static constexpr double kReturnChargeMargin{1.1};

  // Roadmap item 17 stage 5: the light judged from what the navigation sees,
  // and the battery weighed against the way home.
  void onNavigationProgress(const msg::NavigationProgress& progress);

  // Specification K31: the operator's commands. A new goal is the mission's
  // only waypoint from then on; a stop holds where the vehicle is until the
  // next command; home gives the goal up for the start (K20); land is flown
  // by the offboard, and the disarm ends the mission.
  void onOperatorCommand(const std::string& text);

  // The next mission epoch: one past the planner's and past every one the
  // monitor published.
  [[nodiscard]] std::uint64_t nextObjectiveEpoch() noexcept;

  // A position objective on the channel every goal enters by: the return's
  // trail points (K20) and the operator's goals and holds (K31).
  void publishObjective(const Point3& position, const std::uint64_t epoch,
                        const std::uint64_t sample, const bool hold);

  // The return to home (RTH; PX4's own is RTL, which flies no map): the goal
  // is given up for the start, through the channel any objective enters the
  // navigation by; the mission's waypoint list becomes the start
  // alone, so the arrival there is judged as any goal's. A return needs a
  // position source: without a fresh position nothing is substituted.
  [[nodiscard]] bool substituteGoalWithStart(const char* trigger,
                                             const std::int64_t now_ns);

  // The way home is the way the vehicle came (specification K20): the
  // objective is the trail point 25 m back from the one nearest the vehicle,
  // the start at the end.
  void publishReturnObjective();

  [[nodiscard]] double meanSpeed() const noexcept;

  void publishMissionEvent(const std::string& event);

  void report(const bool success, const std::string& reason);

  void finishLanding(const bool disarmed);

  void scheduleShutdown();

  void logSummary();

  Point2 start_{};
  Point2 goal_{};
  Px4MapFrameTransform px4_map_transform_{};
  Point2 latest_position_{};
  std::vector<Point3> waypoints_;
  std::string frame_id_{"map"};
  double spawn_tolerance_m_{1.0};
  double minimum_movement_m_{5.0};
  double acknowledgement_target_tolerance_m_{1.0e-3};
  double goal_capture_radius_m_{2.0};
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
  bool navigation_mission_ready_{false};
  bool shutdown_on_result_{false};
  // Roadmap item 34: the survey's goals come on the operator's channel one
  // after another; reaching one is not the mission's result, coming home is.
  bool survey_{false};
  std::uint64_t navigation_health_producer_instance_id_{0U};
  std::uint64_t navigation_health_sequence_{0U};
  std::uint64_t navigation_mission_epoch_{0U};
  std::size_t proof_voxel_budget_{20'000'000U};
  FlightEnvelopeConfig flight_envelope_{};
  double proof_hold_s_{30.0};
  std::int64_t proven_since_ns_{0};
  Point3 proof_goal_{};
  bool start_unreachable_{false};
  Point3 latest_map_position_{};
  std::int64_t latest_position_stamp_ns_{0};
  std::int64_t mission_start_ns_{0};
  LightReliabilityJudgment light_judgment_;
  double light_charge_s_{std::numeric_limits<double>::quiet_NaN()};
  double route_remaining_m_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Subscription<msg::NavigationProgress>::SharedPtr navigation_progress_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr light_charge_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr light_reading_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr unlit_surface_sub_;
  double light_reading_{std::numeric_limits<double>::quiet_NaN()};
  double unlit_surface_share_{std::numeric_limits<double>::quiet_NaN()};
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_event_pub_;
  double mission_ready_altitude_z_m_{std::numeric_limits<double>::quiet_NaN()};
  double flown_path_m_{0.0};
  bool goal_substituted_{false};
  ReturnTrail return_trail_;
  std::uint64_t return_epoch_{0U};
  std::uint64_t return_sample_{0U};
  std::size_t return_target_{std::numeric_limits<std::size_t>::max()};
  GoalReachabilityProof3D latest_proof_{};
  std::future<GoalReachabilityProof3D> proof_future_;
  std::shared_ptr<const ObservedOccupancyGrid3D> memory_;
  std::uint64_t memory_producer_instance_id_{0U};
  std::uint64_t memory_base_revision_{0U};
  rclcpp::Publisher<msg::NavigationObjective>::SharedPtr objective_pub_;
  rclcpp::Subscription<msg::RawObstacleSnapshot3D>::SharedPtr memory_snapshot_sub_;
  rclcpp::Subscription<msg::RawObstacleDelta3D>::SharedPtr memory_delta_sub_;
  rclcpp::TimerBase::SharedPtr proof_timer_;
  MissionWaypointAcknowledgementAdmissionState waypoint_acknowledgement_admission_{};
  std::unique_ptr<AutopilotStateSource> autopilot_state_source_;
  rclcpp::Subscription<msg::VehicleDestroyed>::SharedPtr vehicle_destroyed_sub_;
  rclcpp::Subscription<msg::MissionWaypointAcknowledgement>::SharedPtr
      waypoint_acknowledgement_sub_;
  rclcpp::Subscription<msg::NavigationHealth>::SharedPtr navigation_health_sub_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
  std::unique_ptr<StepAcknowledger> step_acknowledger_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
  static constexpr double kLandingTimeoutS{90.0};
  MainBatteryEstimate main_battery_;
  bool operator_hold_{false};
  bool operator_landing_{false};
  bool landing_after_result_{false};
  bool landed_reported_{false};
  std::int64_t landing_requested_ns_{0};
  std::uint64_t last_objective_epoch_{0U};
  rclcpp::TimerBase::SharedPtr landing_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operator_command_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operator_command_sub_;
};

} // namespace drone_city_nav
