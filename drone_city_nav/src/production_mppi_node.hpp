#pragma once

#include "drone_city_nav/active_global_guide.hpp"
#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/mppi/mppi_engine.hpp"
#include "drone_city_nav/mppi/passage_speed_policy.hpp"
#include "drone_city_nav/mppi_horizon_safety.hpp"
#include "drone_city_nav/mppi_liveness.hpp"
#include "drone_city_nav/mppi_speed_policy.hpp"
#include "drone_city_nav/msg/mppi_control_feedback.hpp"
#include "drone_city_nav/msg/mppi_trajectory_horizon.hpp"
#include "drone_city_nav/msg/obstacle_memory_snapshot.hpp"
#include "drone_city_nav/msg/raw_obstacle_snapshot.hpp"
#include "drone_city_nav/passage_route_selection.hpp"
#include "drone_city_nav/risk_aware_lattice.hpp"
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

struct ProductionMppiPreparedEsdf {
  std::uint64_t producer_instance_id{0U};
  std::uint64_t revision{0U};
  std::int64_t source_stamp_ns{0};
  std::int64_t ready_stamp_ns{0};
  double build_ms{0.0};
  double conversion_ms{0.0};
  double upload_ms{0.0};
  mppi::EsdfGrid grid{};
  std::shared_ptr<const std::vector<float>> distances_m;
  std::shared_ptr<const std::vector<Point2>> global_guide;
  std::size_t global_guide_expansions{0U};
  double global_guide_cost{0.0};
  std::uint64_t global_guide_generation{0U};
  bool global_guide_reused{false};
  bool global_guide_mission_goal_hold{false};
  GlobalGuideReleaseReason global_guide_release_reason{
      GlobalGuideReleaseReason::kNoActiveGuide};
  GlobalGuideHeadingSource global_guide_heading_source{
      GlobalGuideHeadingSource::kYawFallback};
  GlobalGuideRiskTier global_guide_risk{GlobalGuideRiskTier::kPreferred};
  GlobalGuideProjection global_guide_projection{};
  bool lattice_search_performed{false};
  bool lattice_legacy_valid{false};
  LatticePlanStatus lattice_status{LatticePlanStatus::kInvalidInput};
  LatticeSearchTermination lattice_termination{LatticeSearchTermination::kInvalidInput};
  bool lattice_planning_goal_reached{false};
  double lattice_achieved_progress_m{0.0};
  double lattice_guide_length_m{0.0};
  double lattice_remaining_goal_distance_m{0.0};
  std::size_t lattice_terminal_successor_count{0U};
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

enum class ProductionMppiPlanningState {
  kPlanned,
  kNoGuideBrakingHold,
};

[[nodiscard]] const char*
productionMppiPlanningStateName(ProductionMppiPlanningState state) noexcept;

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
  void esdfWorker(std::stop_token stop_token);
  void planningTick();
  void publishDiagnostics(
      const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
      const ProductionMppiPreparedEsdf& esdf, const ProductionMppiStability& stability,
      const ProductionMppiPredictionError& prediction,
      const MppiLivenessResult& liveness, const MppiSpeedPolicyResult& speed_policy,
      ProductionMppiPlanningState planning_state, std::string_view target_source,
      double pose_age_ms, double esdf_age_ms, double control_feedback_age_ms,
      double snapshot_ms, double stability_ms, double rviz_ms);
  void publishRviz(const mppi::MppiTickInput& input, const mppi::MppiTickResult& result,
                   const ProductionMppiPreparedEsdf& esdf);
  void publishSummary();
  void publishExecutionHorizon(const mppi::MppiTickInput& input,
                               const mppi::MppiTickResult& result,
                               const ProductionMppiPreparedEsdf& esdf,
                               ProductionMppiPlanningState planning_state,
                               std::int64_t now_ns);

  [[nodiscard]] mppi::State selectTarget(const ProductionMppiNavigation& navigation,
                                         const ProductionMppiPreparedEsdf& esdf,
                                         double lookahead_m,
                                         std::string& target_source) const;
  [[nodiscard]] std::optional<mppi::PassageConstraint>
  selectPassageConstraint(const mppi::State& state,
                          std::span<const Point2> guide) const;
  [[nodiscard]] ProductionMppiStability
  compareWithPrevious(const mppi::MppiTickResult& result) const;

  double tick_rate_hz_{50.0};
  double rviz_rate_hz_{10.0};
  double deadline_ms_{20.0};
  double maximum_pose_age_ms_{150.0};
  double maximum_esdf_age_ms_{1000.0};
  double maximum_control_feedback_age_ms_{200.0};
  double no_static_guide_lookahead_m_{30.0};
  PassageRouteSelectionConfig passage_route_selection_config_{};
  Point2 px4_local_origin_{54.0, 54.0};
  Point3 mission_start_{54.0, 54.0, 0.0};
  Point3 mission_goal_{216.0, 378.0, 18.0};
  std::string target_mode_{"active_route_guide"};
  std::string frame_id_{"map"};
  std::filesystem::path diagnostics_output_dir_{"log/mppi"};
  std::int64_t rviz_period_ns_{100000000};
  std::int64_t last_rviz_stamp_ns_{0};

  mppi::BenchmarkConfig mppi_config_{};
  mppi::PassageSpeedPolicy passage_speed_policy_{};
  MppiHorizonSafetyConfig safety_config_{};
  MppiLivenessConfig liveness_config_{};
  MppiSpeedPolicyConfig speed_policy_config_{};
  ActiveGlobalGuideConfig active_guide_config_{};
  GlobalGuideProgressConfig guide_progress_config_{};
  std::unique_ptr<MppiLivenessSupervisor> liveness_supervisor_;
  std::unique_ptr<ActiveGlobalGuideLifecycle> active_guide_lifecycle_;
  std::unique_ptr<GlobalGuideProgressTracker> guide_progress_tracker_;
  RiskAwareLatticeConfig lattice_config_{};
  std::unique_ptr<mppi::MppiCudaEngine> engine_;
  std::optional<KnownPassageMap> known_passages_;
  std::vector<mppi::KnownSolid> known_solids_;

  mutable std::mutex input_mutex_;
  ProductionMppiNavigation navigation_{};
  ProductionMppiAppliedControl applied_control_{};
  std::uint64_t memory_sequence_{0U};
  std::int64_t memory_receive_stamp_ns_{0};

  std::mutex raw_queue_mutex_;
  std::condition_variable_any raw_queue_condition_;
  msg::RawObstacleSnapshot::ConstSharedPtr pending_raw_snapshot_;
  std::uint64_t dropped_raw_snapshots_{0U};
  std::jthread esdf_worker_;
  std::atomic<std::uint64_t> guide_stall_generation_{0U};

  mutable std::mutex esdf_state_mutex_;
  std::optional<ProductionMppiPreparedEsdf> prepared_esdf_;

  std::optional<mppi::MppiTickResult> previous_result_;
  std::optional<mppi::State> previous_predicted_next_state_;
  std::int64_t previous_prediction_stamp_ns_{0};
  ProductionMppiPredictionError latest_prediction_error_{};
  std::uint64_t tick_sequence_{0U};
  std::uint64_t completed_ticks_{0U};
  std::uint64_t deadline_misses_{0U};
  std::uint64_t raw_collision_horizons_{0U};
  std::uint64_t solid_collision_horizons_{0U};
  std::uint64_t post_update_contract_violations_{0U};
  std::uint64_t no_progress_horizons_{0U};
  std::uint64_t liveness_reseeds_{0U};
  std::uint64_t no_guide_braking_hold_ticks_{0U};
  std::vector<double> runtime_samples_ms_;
  std::int64_t last_summary_stamp_ns_{0};
  std::ofstream diagnostics_stream_;

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
