#pragma once

#include "drone_city_nav/active_global_guide.hpp"
#include "drone_city_nav/bounded_worker_pool.hpp"
#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/global_guide_candidate.hpp"
#include "drone_city_nav/latest_value_mailbox.hpp"
#include "drone_city_nav/mission_goal_capture.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi_horizon_safety.hpp"
#include "drone_city_nav/mppi_liveness.hpp"
#include "drone_city_nav/mppi_nominal_reseed.hpp"
#include "drone_city_nav/mppi_risk_escalation.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/msg/mppi_control_feedback.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/obstacle_memory_snapshot.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/navigation_state_prediction.hpp"
#include "drone_city_nav/no_static_route_cycle.hpp"
#include "drone_city_nav/raw_guide_validation.hpp"
#include "drone_city_nav/risk_aware_lattice.hpp"
#include "drone_city_nav/risk_aware_lattice_3d.hpp"
#include "drone_city_nav/route_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/static_route_geometry.hpp"
#include "drone_city_nav/types.hpp"

#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

namespace drone_city_nav {

struct ProductionMppiNavigation {
  mppi::State state{};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t revision{0U};
  bool valid{false};
};

enum class ProductionPlanningSearchKind : std::uint8_t {
  kNone,
  kLattice2D,
  kLattice3D,
};

enum class ProductionGuideCandidateValidationStatus : std::uint8_t {
  kNotAttempted,
  kAccepted,
  kUnavailableLatestWorld,
  kInvalidProjection,
  kExcessiveCrossTrack,
  kRawValidationRejected,
  kLifecycleRejected,
};

struct ProductionMppiPreparedEsdf {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t revision{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t ready_stamp_ns{0};
  double build_ms{0.0};
  double esdf_x_pass_ms{0.0};
  double esdf_y_pass_ms{0.0};
  double esdf_z_pass_ms{0.0};
  double esdf_finalize_ms{0.0};
  double conversion_ms{0.0};
  double upload_ms{0.0};
  double global_guide_search_ms{0.0};
  double continuation_validation_ms{0.0};
  double route_smoothing_ms{0.0};
  std::size_t route_shortcuts_applied{0U};
  std::size_t route_corners_smoothed{0U};
  double candidate_validation_ms{0.0};
  std::uint64_t route_fingerprint{0U};
  mppi::EsdfGrid grid{};
  std::shared_ptr<const std::vector<float>> distances_m;
  std::shared_ptr<const OccupancyGrid2D> raw_occupancy;
  std::shared_ptr<const std::vector<mppi::RouteSample3D>> mppi_route;
  std::shared_ptr<const std::vector<RouteSample3D>> route_3d;
  std::shared_ptr<const std::vector<Point2>> route_2d_projection;
  std::shared_ptr<const std::vector<ConstrainedRouteSpan>> constrained_spans;
  std::shared_ptr<const std::vector<ConstrainedFreeSpaceEdge>> channel_edges;
  std::shared_ptr<const std::vector<std::string>> selected_channel_ids;
  std::vector<Lattice3DTopologyCandidate> topology_candidates;
  double topology_objective_cost{0.0};
  double topology_route_length_m{0.0};
  double topology_travel_time_s{0.0};
  double topology_vertical_alignment_time_s{0.0};
  double topology_planning_exposure_m{0.0};
  double topology_critical_exposure_m{0.0};
  std::size_t global_guide_expansions{0U};
  double global_guide_cost{0.0};
  std::uint64_t global_guide_generation{0U};
  bool global_guide_reused{false};
  bool global_guide_reaches_mission_goal{false};
  GlobalGuideReleaseReason global_guide_release_reason{
      GlobalGuideReleaseReason::kNoActiveGuide};
  GlobalGuideHeadingSource global_guide_heading_source{
      GlobalGuideHeadingSource::kGoalDirection};
  GlobalGuideRiskTier global_guide_risk{GlobalGuideRiskTier::kPreferred};
  GlobalGuideAcceptanceReason global_guide_acceptance_reason{
      GlobalGuideAcceptanceReason::kNotAttempted};
  GlobalGuideProjection global_guide_projection{};
  ProductionPlanningSearchKind planning_search_kind{
      ProductionPlanningSearchKind::kNone};
  Point3 planning_search_start{};
  Point3 planning_search_goal{};
  Point3 planning_candidate_endpoint{};
  Vec3 planning_search_direction{};
  std::size_t planning_candidate_points{0U};
  std::size_t planning_candidate_samples{0U};
  bool lattice_search_performed{false};
  bool lattice_executable{false};
  LatticePlanStatus lattice_status{LatticePlanStatus::kInvalidInput};
  LatticeSearchTermination lattice_termination{LatticeSearchTermination::kInvalidInput};
  bool lattice_planning_goal_reached{false};
  double lattice_achieved_progress_m{0.0};
  double lattice_guide_length_m{0.0};
  double lattice_remaining_goal_distance_m{0.0};
  std::size_t lattice_terminal_successor_count{0U};
  LatticeRiskStage lattice_risk_stage{LatticeRiskStage::kPreferredOnly};
  std::size_t lattice_stale_queue_pops{0U};
  std::size_t lattice_open_peak{0U};
  std::size_t lattice_records_peak{0U};
  std::size_t lattice_continuation_reachable_states{0U};
  double lattice_reachable_depth_m{0.0};
  double lattice_frontier_endpoint_displacement_m{0.0};
  double lattice_frontier_selection_score{0.0};
  std::size_t lattice_frontier_candidates_considered{0U};
  LatticeSuccessorDiagnostics lattice_successor_diagnostics{};
  LatticeSuccessorProfiling lattice_successor_profiling{};
  Lattice3DStatus lattice_3d_status{Lattice3DStatus::kInvalidInput};
  Lattice3DRiskStage lattice_3d_risk_stage{Lattice3DRiskStage::kPreferredOnly};
  Lattice3DSearchTermination lattice_3d_termination{
      Lattice3DSearchTermination::kInvalidInput};
  double lattice_3d_minimum_clearance_m{0.0};
  Lattice3DSuccessorDiagnostics lattice_3d_successor_diagnostics{};
  Lattice3DSuccessorProfiling lattice_3d_successor_profiling{};
  std::size_t lattice_continuation_attempt{0U};
  double lattice_search_session_age_ms{0.0};
  bool no_static_cycle_detected{false};
  bool no_static_adaptive_search{false};
  std::size_t no_static_soft_tabu_entries{0U};
  bool lattice_search_session_resumed{false};
  bool lattice_search_session_complete{true};
  std::uint64_t lattice_search_revision{0U};
  std::uint64_t lattice_validation_revision{0U};
  RawGuideValidationStatus lattice_raw_validation_status{
      RawGuideValidationStatus::kInvalidGuide};
  ProductionGuideCandidateValidationStatus guide_candidate_validation_status{
      ProductionGuideCandidateValidationStatus::kNotAttempted};
  bool static_route_extension_request{false};
  std::uint64_t static_route_extension_base_generation{0U};
  bool static_route_replan_request{false};
  std::uint64_t static_route_replan_base_generation{0U};
  GlobalGuideReleaseReason static_route_replan_reason{GlobalGuideReleaseReason::kNone};
  StaticRouteCandidateStatus static_route_candidate_status{
      StaticRouteCandidateStatus::kEmpty};
  StaticRouteActivationStatus static_route_activation_status{
      StaticRouteActivationStatus::kNotAttempted};
  bool static_route_revision_matches{false};
  bool static_route_generation_matches{false};
};

struct ProductionMppiStability {
  double first_control_delta{0.0};
  double position_rms_m{0.0};
  double position_max_m{0.0};
  double terminal_shift_m{0.0};
  bool valid{false};
};

struct ProductionMppiPredictionError {
  double position_m{0.0};
  double velocity_mps{0.0};
  double yaw_rad{0.0};
  bool valid{false};
};

struct ProductionMppiAppliedControl {
  mppi::Control control{};
  std::int64_t receive_stamp_ns{0};
  std::uint64_t horizon_sequence{0U};
  bool emergency_braking{false};
  bool valid{false};
};

struct ProductionMppiRvizSnapshot {
  std::vector<mppi::State> candidate_horizon;
  std::vector<mppi::State> previous_horizon;
  std::vector<mppi::State> execution_horizon;
  std::shared_ptr<const std::vector<mppi::RouteSample3D>> route;
  std::shared_ptr<const std::vector<ConstrainedFreeSpaceEdge>> channel_edges;
  std::shared_ptr<const std::vector<std::string>> selected_channel_ids;
};

enum class ProductionMppiExecutionMode : std::uint8_t {
  kPlanned,
  kBraking,
  kPositionHold,
};

enum class ProductionMppiExecutionReason : std::uint8_t {
  kNone,
  kHorizonSafety,
  kGoalCapture,
  kNoGuide,
  kUnavailableWorld,
};

struct ProductionMppiExecutionPublication {
  std::vector<mppi::State> horizon;
  ProductionMppiExecutionMode mode{ProductionMppiExecutionMode::kPlanned};
  ProductionMppiExecutionReason reason{ProductionMppiExecutionReason::kNone};
  bool published{false};
};

enum class ProductionMppiPlanningState {
  kPlanned,
  kNoGuideBrakingHold,
  kUnavailableWorldBrakingHold,
  kMissionGoalPositionHold,
};

struct ProductionMppiDiagnosticsSnapshot {
  mppi::MppiTickInput input{};
  mppi::MppiTickResult result{};
  ProductionMppiPreparedEsdf esdf{};
  ProductionMppiStability stability{};
  ProductionMppiPredictionError prediction{};
  MppiLivenessResult liveness{};
  MppiSpeedPolicyResult speed_policy{};
  GlobalGuideProgressUpdate guide_progress{};
  MppiEligibleRolloutUpdate no_eligible_recovery{};
  MissionGoalCaptureResult goal_capture{};
  ProductionMppiExecutionPublication execution{};
  ProductionMppiPlanningState planning_state{ProductionMppiPlanningState::kPlanned};
  std::optional<ProductionMppiRvizSnapshot> rviz;
  std::string target_source;
  std::uint64_t tick_sequence{0U};
  std::uint64_t memory_sequence{0U};
  double pose_age_ms{0.0};
  double esdf_age_ms{0.0};
  double control_feedback_age_ms{0.0};
  double route_station_m{0.0};
  double route_remaining_m{0.0};
  double snapshot_ms{0.0};
  double stability_ms{0.0};
  bool route_projection_valid{false};
  bool liveness_reseed_requested{false};
  bool pose_predicted{false};
  mppi::RiskTier route_required_risk_tier{mppi::RiskTier::kPreferred};
  mppi::RiskTier maximum_eligible_risk_tier{mppi::RiskTier::kPreferred};
};

[[nodiscard]] const char*
productionMppiPlanningStateName(ProductionMppiPlanningState state) noexcept;
[[nodiscard]] const char*
productionMppiExecutionModeName(ProductionMppiExecutionMode mode) noexcept;
[[nodiscard]] const char*
productionMppiExecutionReasonName(ProductionMppiExecutionReason reason) noexcept;
[[nodiscard]] const char*
productionPlanningSearchKindName(ProductionPlanningSearchKind kind) noexcept;
[[nodiscard]] const char* productionGuideCandidateValidationStatusName(
    ProductionGuideCandidateValidationStatus status) noexcept;

class ProductionMppiNode final : public rclcpp::Node {
public:
  ProductionMppiNode();
  ~ProductionMppiNode() override;

  ProductionMppiNode(const ProductionMppiNode&) = delete;
  ProductionMppiNode& operator=(const ProductionMppiNode&) = delete;
  ProductionMppiNode(ProductionMppiNode&&) = delete;
  ProductionMppiNode& operator=(ProductionMppiNode&&) = delete;

private:
  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& message);
  void onRawObstacleSnapshot(msg::RawObstacleSnapshot::ConstSharedPtr message);
  void onMemorySnapshot(const msg::ObstacleMemorySnapshot& message);
  void onAppliedControl(const msg::MppiControlFeedback& message);
  void requestGuideRelease(GlobalGuideReleaseReason reason,
                           std::uint64_t guide_generation = 0U);
  void maybeRequestStaticRouteExtension(const ProductionMppiPreparedEsdf& esdf,
                                        const ProductionMppiNavigation& navigation,
                                        const GlobalGuideProjection& route_projection,
                                        std::int64_t now_ns);
  void finishStaticRouteExtension(std::uint64_t base_generation) noexcept;
  void finishStaticRouteReplan(std::uint64_t base_generation) noexcept;
  void esdfWorker(std::stop_token stop_token);
  void guideWorker(std::stop_token stop_token);
  void processStaticGuideSearch(const ProductionMppiPreparedEsdf& world,
                                const ProductionMppiNavigation& navigation);
  void diagnosticsWorker(std::stop_token stop_token);
  void planningTick();
  void processDiagnostics(const ProductionMppiDiagnosticsSnapshot& snapshot);
  void publishRviz(const ProductionMppiDiagnosticsSnapshot& snapshot);
  void enqueueDiagnostics(ProductionMppiDiagnosticsSnapshot snapshot);
  void recordTickStatistics(const mppi::MppiTickResult& result,
                            ProductionMppiPlanningState planning_state,
                            bool liveness_reseed_requested);
  void publishSummary();
  [[nodiscard]] ProductionMppiExecutionPublication publishExecutionHorizon(
      const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
      const ProductionMppiPreparedEsdf& esdf,
      ProductionMppiPlanningState planning_state, std::int64_t now_ns);

  [[nodiscard]] mppi::State selectTarget(const ProductionMppiPreparedEsdf& esdf,
                                         double current_station_m, double lookahead_m,
                                         std::string& target_source,
                                         double& target_station_m) const;
  [[nodiscard]] ProductionMppiStability
  compareWithPrevious(const mppi::MppiTickResult& result) const;

  double tick_rate_hz_{50.0};
  double rviz_rate_hz_{10.0};
  double diagnostics_info_rate_hz_{5.0};
  double deadline_ms_{20.0};
  double maximum_pose_age_ms_{150.0};
  double maximum_pose_prediction_age_ms_{1000.0};
  double maximum_esdf_age_ms_{1000.0};
  double stale_esdf_execution_window_ms_{4000.0};
  double maximum_control_feedback_age_ms_{200.0};
  double no_static_guide_lookahead_m_{30.0};
  bool frontier_blacklist_enabled_{false};
  double frontier_blacklist_ttl_s_{15.0};
  double no_static_soft_tabu_penalty_{40.0};
  double no_static_soft_tabu_sample_spacing_m_{4.0};
  double no_static_adaptive_reachable_depth_m_{40.0};
  double no_static_adaptive_minimum_guide_length_m_{24.0};
  double no_static_adaptive_minimum_endpoint_displacement_m_{12.0};
  std::size_t no_static_adaptive_validation_states_{8192U};
  std::size_t planner_worker_count_{4U};
  NoStaticRouteCycleConfig no_static_cycle_config_{};
  MissionGoalCaptureConfig mission_goal_capture_config_{};
  Point2 px4_local_origin_{54.0, 54.0};
  Point3 mission_start_{54.0, 54.0, 0.0};
  Point3 mission_goal_{216.0, 378.0, 18.0};
  std::string target_mode_{"active_route_guide"};
  bool use_static_map_{true};
  float constrained_route_speed_limit_mps_{10.0F};
  double route_constraint_diagnostics_distance_m_{30.0};
  std::string frame_id_{"map"};
  std::filesystem::path diagnostics_output_dir_{"log/mppi"};
  std::int64_t rviz_period_ns_{100000000};
  std::int64_t diagnostics_info_period_ns_{200000000};
  std::int64_t last_rviz_stamp_ns_{0};
  std::int64_t last_diagnostics_info_stamp_ns_{0};
  std::optional<ConstrainedRouteObservation> last_route_constraint_observation_;

  mppi::BenchmarkConfig mppi_config_{};
  MppiHorizonSafetyConfig safety_config_{};
  MppiSafetyInterventionTracker safety_intervention_tracker_{};
  MppiBrakeHoldLifecycle brake_hold_lifecycle_{};
  MppiLivenessConfig liveness_config_{};
  MppiSpeedPolicyConfig speed_policy_config_{};
  ActiveGlobalGuideConfig active_guide_config_{};
  GlobalGuideProgressConfig guide_progress_config_{};
  std::unique_ptr<MppiLivenessSupervisor> liveness_supervisor_;
  std::unique_ptr<MppiRiskEscalation> risk_escalation_;
  MppiNominalReseedTracker nominal_reseed_tracker_{};
  std::unique_ptr<ActiveGlobalGuideLifecycle> active_guide_lifecycle_;
  std::unique_ptr<GlobalGuideProgressTracker> guide_progress_tracker_;
  std::unique_ptr<MissionGoalCaptureLatch> mission_goal_capture_latch_;
  std::unique_ptr<NoStaticRouteCycleDetector> no_static_cycle_detector_;
  RiskAwareLatticeConfig lattice_config_{};
  RiskAwareLattice3DConfig lattice_3d_config_{};
  RouteEnvelopeConfig route_envelope_config_{};
  ConstrainedRouteControlConfig constrained_route_control_config_{};
  ConstrainedRouteCoordinator constrained_route_coordinator_{};
  StaticRouteExtensionConfig static_route_extension_config_{};
  StaticRouteGeometryConfig static_route_geometry_config_{};
  std::unique_ptr<BoundedWorkerPool> planning_worker_pool_;
  std::unique_ptr<mppi::MppiCudaEngine> engine_;
  std::optional<OccupancyGrid3D> static_occupancy_3d_;
  std::shared_ptr<const std::vector<ConstrainedFreeSpaceEdge>> static_channel_edges_;
  std::shared_ptr<const std::vector<float>> static_esdf_3d_;
  mppi::EsdfGrid static_esdf_grid_{};
  std::uint64_t static_route_generation_{0U};
  std::uint64_t tracked_route_generation_{0U};
  double tracked_route_station_m_{0.0};
  std::mutex static_route_extension_mutex_;
  bool static_route_extension_request_in_flight_{false};
  std::uint64_t static_route_extension_in_flight_generation_{0U};
  std::uint64_t static_route_extension_last_request_generation_{0U};
  double static_route_extension_last_request_station_m_{0.0};
  std::int64_t static_route_extension_last_request_stamp_ns_{0};
  StaticRouteReplanGate static_route_replan_gate_{};
  StaticRouteRoiRefreshLifecycle static_roi_refresh_lifecycle_{};

  mutable std::mutex input_mutex_;
  ProductionMppiNavigation navigation_{};
  ProductionMppiAppliedControl applied_control_{};
  std::uint64_t memory_sequence_{0U};
  std::int64_t memory_receive_stamp_ns_{0};

  std::mutex raw_queue_mutex_;
  std::condition_variable_any raw_queue_condition_;
  msg::RawObstacleSnapshot::ConstSharedPtr pending_raw_snapshot_;
  std::atomic<std::uint64_t> dropped_raw_snapshots_{0U};
  std::jthread esdf_worker_;
  std::mutex guide_queue_mutex_;
  std::condition_variable_any guide_queue_condition_;
  std::shared_ptr<const ProductionMppiPreparedEsdf> pending_guide_world_;
  std::atomic<std::uint64_t> dropped_guide_worlds_{0U};
  std::jthread guide_worker_;
  std::atomic<std::uint64_t> guide_release_generation_{0U};
  std::atomic<GlobalGuideReleaseReason> guide_release_reason_{
      GlobalGuideReleaseReason::kStalled};
  std::optional<GlobalGuideCandidate> pending_global_guide_;
  std::vector<LatticeFrontierBlacklistEntry> frontier_blacklist_;

  mutable std::mutex esdf_state_mutex_;
  std::optional<ProductionMppiPreparedEsdf> prepared_esdf_;

  std::optional<mppi::MppiTickResult> previous_result_;
  std::optional<mppi::State> previous_predicted_next_state_;
  std::int64_t previous_prediction_stamp_ns_{0};
  ProductionMppiPredictionError latest_prediction_error_{};
  std::uint64_t tick_sequence_{0U};
  mppi::RiskTier maximum_eligible_risk_tier_{mppi::RiskTier::kPreferred};
  std::uint64_t completed_ticks_{0U};
  std::uint64_t deadline_misses_{0U};
  std::uint64_t raw_collision_horizons_{0U};
  std::uint64_t solid_collision_horizons_{0U};
  std::uint64_t post_update_contract_violations_{0U};
  std::uint64_t no_progress_horizons_{0U};
  std::uint64_t liveness_reseeds_{0U};
  std::uint64_t no_guide_braking_hold_ticks_{0U};
  std::uint64_t unavailable_world_braking_hold_ticks_{0U};
  std::uint64_t mission_goal_position_hold_ticks_{0U};
  std::vector<double> runtime_samples_ms_;
  std::int64_t last_summary_stamp_ns_{0};
  mutable std::mutex statistics_mutex_;
  std::ofstream diagnostics_stream_;
  LatestValueMailbox<ProductionMppiDiagnosticsSnapshot> diagnostics_mailbox_;
  std::atomic<std::uint64_t> dropped_diagnostics_snapshots_{0U};
  std::jthread diagnostics_worker_;

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<msg::RawObstacleSnapshot>::SharedPtr raw_snapshot_sub_;
  rclcpp::Subscription<msg::ObstacleMemorySnapshot>::SharedPtr memory_snapshot_sub_;
  rclcpp::Subscription<msg::MppiControlFeedback>::SharedPtr applied_control_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<msg::MppiTrajectoryHorizon>::SharedPtr execution_horizon_pub_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
};

} // namespace drone_city_nav
