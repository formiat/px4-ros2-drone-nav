#pragma once

#include "drone_city_nav/corridor_samples_io.hpp"
#include "drone_city_nav/current_lidar_overlay.hpp"
#include "drone_city_nav/grid_overlay.hpp"
#include "drone_city_nav/known_passage_debug_markers.hpp"
#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"
#include "drone_city_nav/lidar_motion_compensation.hpp"
#include "drone_city_nav/lidar_pose_history.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/msg/replan_blocker_event.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory_provenance_ros.hpp"
#include "drone_city_nav/path_smoothing.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/planner_diagnostics_format.hpp"
#include "drone_city_nav/planner_node_config.hpp"
#include "drone_city_nav/planner_path_publication.hpp"
#include "drone_city_nav/planner_runtime_state.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"
#include "drone_city_nav/px4_ros_time_mapper.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/static_map_debug.hpp"
#include "drone_city_nav/static_map_source.hpp"
#include "drone_city_nav/trajectory_diagnostics_io.hpp"
#include "drone_city_nav/trajectory_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <array>
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <numbers>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace drone_city_nav {

[[nodiscard]] inline bool finite2D(const Point2 point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

[[nodiscard]] inline double radiansToDegrees(const double radians) noexcept {
  return radians * 180.0 / std::numbers::pi;
}

inline constexpr double kPublishedPathCollinearityToleranceM = 0.05;
inline constexpr double kGroundDebugZ = 0.05;

[[nodiscard]] inline std::uint64_t
stampNanoseconds(const builtin_interfaces::msg::Time& stamp) {
  constexpr std::uint64_t kNanosecondsPerSecond = 1'000'000'000U;
  return static_cast<std::uint64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<std::uint64_t>(stamp.nanosec);
}

[[nodiscard]] inline double
elapsedMilliseconds(const std::chrono::steady_clock::time_point start) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(
                                 std::chrono::steady_clock::now() - start)
                                 .count()) /
         1000.0;
}

[[nodiscard]] inline std::vector<Point2>
trajectorySamplePoints(const std::span<const TrajectoryPointSample> samples) {
  std::vector<Point2> points;
  points.reserve(samples.size());
  for (const TrajectoryPointSample& sample : samples) {
    points.push_back(sample.point);
  }
  return points;
}

class PlannerNode final : public rclcpp::Node {
public:
  PlannerNode();
  ~PlannerNode() override;

private:
  struct NavigationStateSnapshot {
    Pose2 pose{};
    Point2 velocity{};
    AttitudeEuler attitude{};
    double altitude_m{std::numeric_limits<double>::quiet_NaN()};
    double speed_mps{std::numeric_limits<double>::quiet_NaN()};
    std::int64_t stamp_ns{0};
    bool pose_valid{false};
    bool altitude_valid{false};
    bool velocity_valid{false};
    bool attitude_valid{false};
  };

  struct LidarInputSnapshot {
    sensor_msgs::msg::LaserScan scan;
    LidarProjectionPose projection_pose{};
    std::vector<LidarProjectionPose> beam_projection_poses;
    LidarProjectionPoseSource projection_pose_source{
        LidarProjectionPoseSource::kCallbackPoseFallback};
    Point2 motion_shift{};
    double pose_lag_s{0.0};
    double pose_latency_s{0.0};
    double motion_shift_m{0.0};
    std::int64_t update_ns{0};
    bool seen{false};
    bool projection_pose_valid{false};
  };

  struct PendingMemorySnapshot {
    OccupancyGrid2D grid;
    MemoryProvenanceSnapshot provenance;
    std::uint64_t producer_instance_id{0U};
    std::uint64_t sequence{0U};
    std::uint64_t producer_assembly_duration_ns{0U};
    std::int64_t stamp_ns{0};
    std::int64_t receive_ns{0};
    double receive_age_ms{std::numeric_limits<double>::quiet_NaN()};
    double callback_ms{std::numeric_limits<double>::quiet_NaN()};
  };

  void applyConfig(const PlannerNodeConfig& config);

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg);

  void onMemorySnapshot(msg::ObstacleMemorySnapshot::ConstSharedPtr message);

  void applyPendingMemorySnapshot(std::int64_t now_ns);

  void onScan(const sensor_msgs::msg::LaserScan& msg);

  void onAttitude(const px4_msgs::msg::VehicleAttitude& msg);

  void onTimesyncStatus(const px4_msgs::msg::TimesyncStatus& msg);

  [[nodiscard]] std::filesystem::path staticMapPackageShareDirectory() const;

  void loadConfiguredStaticMap();

  void loadConfiguredKnownPassages();

  [[nodiscard]] PlanningGridBuilderConfig planningGridBuilderConfig() const;

  [[nodiscard]] std::optional<PlanningGridBuildResult>
  buildPlanningGrid(const std::int64_t now_ns);

  void checkCurrentPathAndPublish();

  void runPlanningCycle(std::uint64_t request_generation);

  void planningWorkerLoop(std::stop_token stop_token);

  void requestPlanningCycle();

  [[nodiscard]] NavigationStateSnapshot navigationStateSnapshot() const;

  void applyNavigationStateSnapshot(const NavigationStateSnapshot& snapshot);

  void applyLatestLidarInputSnapshot();

  [[nodiscard]] Point2 predictedPlanningStart(const NavigationStateSnapshot& navigation,
                                              double horizon_s) const;

  [[nodiscard]] AStarConfig astarConfigForCurrentVelocity() const;

  [[nodiscard]] static bool
  initialHeadingBiasActive(const AStarConfig& config) noexcept;

  [[nodiscard]] TrajectoryPlannerConfig
  trajectoryPlannerConfigForCurrentAltitude() const;

  [[nodiscard]] std::optional<PathComputationResult>
  computePathOnGrid(const OccupancyGrid2D& grid, const char* source_label,
                    const AStarConfig& astar_config, Point2 planning_start);

  bool publishPathFromPathCells(const OccupancyGrid2D& route_grid,
                                const std::vector<GridIndex>& raw_cells,
                                const std::vector<GridIndex>& smoothed_cells,
                                const char* source_label,
                                const ClearanceField2D* route_clearance_field,
                                bool route_clearance_field_cache_hit,
                                Point2 planning_start);

  bool publishTrajectoryResult(const TrajectoryPlannerResult& trajectory_result,
                               std::span<const Point2> route_points,
                               const char* source_label, double duration_ms,
                               TrajectoryDeliveryDiagnostics delivery,
                               std::uint64_t* published_path_id = nullptr);

  [[nodiscard]] bool
  keepCurrentPathAfterInvalidReplacement(const char* source_label,
                                         const char* invalid_reason) const;

  [[nodiscard]] PublishedPathSafetySummary
  summarizePublishedPathSafety(const OccupancyGrid2D& grid,
                               const std::span<const Point2> path_points) const;

  void logPublishedPathSafety(const OccupancyGrid2D& grid,
                              const std::span<const Point2> path_points,
                              const char* source_label) const;

  [[nodiscard]] bool connectRouteToCurrentPose(const OccupancyGrid2D& grid,
                                               std::vector<Point2>& path_points,
                                               const char* source_label,
                                               Point2 planning_start) const;

  void logRejectedUnsafeRoute(const OccupancyGrid2D& grid,
                              const std::span<const Point2> path_points,
                              const char* source_label, const char* reason) const;

  [[nodiscard]] double currentLidarRangeMax() const;

  [[nodiscard]] double
  currentLidarPoseReceiveLagSeconds(const std::int64_t scan_receive_ns,
                                    const std::int64_t pose_receive_ns) const;

  [[nodiscard]] LidarProjectionPose currentLidarProjectionPose() const;

  [[nodiscard]] LidarProjectionConfig currentLidarProjectionConfig() const;

  CurrentLidarOverlayStats overlayCurrentLidarHits(OccupancyGrid2D& grid,
                                                   const std::int64_t now_ns) const;

  [[nodiscard]] std_msgs::msg::Header makePlannerHeader() const;

  void publishStaticMapDebug(const OccupancyGrid2D& grid, const bool log_publication);

  void republishStaticMapDebug();

  void publishKnownPassageDebug(const bool log_publication);

  void republishKnownPassageDebug();

  void publishProhibitedGrid(const OccupancyGrid2D& grid);

  std::uint64_t publishPath(const std::vector<Point2>& points,
                            PathPublicationReason reason,
                            const TrajectoryPlannerStats* trajectory_stats = nullptr);

  std::uint64_t
  publishTrajectoryPath(std::span<const TrajectoryPointSample> samples,
                        PathPublicationReason reason,
                        const TrajectoryPlannerStats* trajectory_stats = nullptr,
                        TrajectoryDeliveryDiagnostics delivery = {});

  void
  publishTrajectoryDiagnostics(const std::uint64_t path_id,
                               const std::uint64_t path_stamp_ns,
                               const TrajectoryPlannerStats& stats,
                               const TrajectoryDeliveryDiagnostics& delivery) const;

  [[nodiscard]] static std::filesystem::path corridorSamplesDirectory();

  bool writeCorridorSamplesCsvFile(const std::filesystem::path& path,
                                   const TrajectoryPlannerResult& result,
                                   const char* source_label,
                                   const std::uint64_t candidate_path_id) const;

  void writeCorridorSamplesDump(const TrajectoryPlannerResult& result,
                                const char* source_label,
                                const std::uint64_t candidate_path_id) const;

  [[nodiscard]] static std::filesystem::path trajectoryCandidatesDirectory();

  bool writeTrajectoryOptimizerCandidatesCsvFile(const std::filesystem::path& path,
                                                 const TrajectoryPlannerResult& result,
                                                 const char* source_label,
                                                 std::uint64_t candidate_path_id) const;

  bool writeTurnSmoothingCandidatesCsvFile(const std::filesystem::path& path,
                                           const TrajectoryPlannerResult& result,
                                           const char* source_label,
                                           std::uint64_t candidate_path_id) const;

  void writeTrajectoryCandidateDumps(const TrajectoryPlannerResult& result,
                                     const char* source_label,
                                     std::uint64_t candidate_path_id) const;

  void publishPlanningFailureHold();

  void invalidateCurrentPose();

  [[nodiscard]] double poseAgeSeconds(const std::int64_t now_ns) const;

  [[nodiscard]] double scanAgeSeconds(const std::int64_t now_ns) const;

  void logMemorySnapshotTransportSummary(std::int64_t now_ns);

  [[nodiscard]] std::string
  memorySnapshotTransportDiagnostic(std::int64_t now_ns) const;

  [[nodiscard]] std::string
  describeProhibitedIntersectionSource(const OccupancyGrid2D& grid,
                                       const PathProhibitedIntersection& intersection,
                                       const PlanningGridBuildResult& planning_result);

  bool keepCurrentPathIfStillClear(const OccupancyGrid2D& grid,
                                   const PlanningGridBuildResult& planning_result);

  void logPathUpdate(const nav_msgs::msg::Path& path, const PathMetrics& metrics,
                     const PathPublicationReason reason, const std::uint64_t path_id);

  void recordPathPublication(const PathPublicationReason reason, const bool empty_path);

  [[nodiscard]] std::string plannerCountersSummary() const;

  void logPlannerCountersThrottled();

  std::optional<OccupancyGrid2D> memory_grid_;
  std::optional<MemoryProvenanceSnapshot> memory_provenance_snapshot_;
  std::optional<OccupancyGrid2D> static_grid_;
  std::optional<StaticCityMap> static_map_debug_;
  std::optional<KnownPassageMap> known_passages_;
  std::optional<KnownStaticLidarHitClassifier> known_static_lidar_classifier_;
  PlanningGridBuilder planning_grid_builder_;
  PlannerCore planner_core_;
  AStarConfig astar_config_{};
  TrajectoryPlannerConfig trajectory_planner_config_{};

  Pose2 current_pose_{};
  Point2 current_velocity_{};
  AttitudeEuler current_attitude_{};
  LidarProjectionPose last_scan_projection_pose_{};
  LidarProjectionPoseSource last_scan_projection_pose_source_{
      LidarProjectionPoseSource::kCallbackPoseFallback};
  LidarPoseHistory lidar_pose_history_{};
  Px4RosTimeMapper px4_ros_time_mapper_{};
  mutable AmbiguousLidarHitTracker current_lidar_ambiguous_hit_tracker_{};
  Point2 start_{};
  Point2 goal_{};
  Point2 px4_local_origin_{};
  sensor_msgs::msg::LaserScan last_scan_;
  bool pose_valid_{false};
  bool local_position_seen_{false};
  bool memory_grid_seen_{false};
  bool scan_seen_{false};
  bool scan_seen_logged_{false};
  bool use_static_map_{true};
  bool safe_trajectory_truncation_enabled_{true};
  bool use_known_passages_{true};
  bool use_px4_heading_for_scan_{true};
  bool motion_compensate_lidar_pose_{true};
  bool compensate_lidar_attitude_{true};
  bool altitude_valid_{false};
  bool attitude_valid_{false};
  bool current_velocity_valid_{false};
  bool last_scan_projection_pose_valid_{false};
  std::string frame_id_{"map"};
  std::string static_map_path_param_{"worlds/generated_city.map2d"};
  std::string known_passages_path_param_{"worlds/known_passages.passages3d"};
  std::filesystem::path static_map_resolved_path_;
  std::filesystem::path known_passages_resolved_path_;
  GridBounds fallback_grid_bounds_{-10.0, -10.0, 0.5, 230, 350};
  double inflation_radius_m_{1.0};
  double planning_clearance_m_{3.0};
  double local_inflation_relaxation_radius_m_{5.0};
  double initial_altitude_m_{12.0};
  double static_map_min_blocking_height_m_{0.0};
  double stable_path_goal_tolerance_m_{3.0};
  double max_lidar_range_m_{35.0};
  double range_hit_epsilon_m_{0.05};
  double scan_yaw_offset_rad_{0.0};
  double initial_heading_rad_{0.0};
  double static_map_debug_publish_period_s_{1.0};
  double known_passage_debug_publish_period_s_{1.0};
  KnownPassageValidationConfig known_passage_validation_config_{};
  GroundLidarRejectionConfig ground_lidar_rejection_config_{};
  LidarIngestionConfidenceConfig lidar_ingestion_confidence_config_{};
  double known_static_lidar_hit_closer_range_tolerance_m_{0.5};
  double known_static_lidar_hit_farther_range_tolerance_m_{1.5};
  double known_static_lidar_hit_endpoint_volume_tolerance_m_{0.75};
  double known_static_opening_boundary_tolerance_m_{0.30};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double current_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_scan_pose_lag_s_{0.0};
  double last_scan_pose_latency_s_{0.0};
  double last_scan_motion_shift_m_{0.0};
  double lidar_pose_latency_s_{0.05};
  double memory_snapshot_diagnostic_period_s_{5.0};
  double memory_snapshot_max_age_ms_{350.0};
  double memory_snapshot_max_callback_time_ms_{100.0};
  double memory_snapshot_max_apply_delay_ms_{300.0};
  double memory_snapshot_min_apply_rate_hz_{1.0};
  double last_memory_snapshot_age_ms_{std::numeric_limits<double>::quiet_NaN()};
  double last_memory_snapshot_receive_age_ms_{std::numeric_limits<double>::quiet_NaN()};
  double last_memory_snapshot_callback_ms_{std::numeric_limits<double>::quiet_NaN()};
  double last_memory_snapshot_interval_ms_{std::numeric_limits<double>::quiet_NaN()};
  double last_memory_snapshot_apply_delay_ms_{std::numeric_limits<double>::quiet_NaN()};
  double last_memory_snapshot_apply_rate_hz_{0.0};
  double last_memory_snapshot_receive_rate_hz_{0.0};
  double memory_snapshot_max_age_since_report_ms_{0.0};
  double memory_snapshot_max_callback_since_report_ms_{0.0};
  double memory_snapshot_max_apply_delay_since_report_ms_{0.0};
  Point2 last_scan_motion_shift_{};
  double lidar_z_offset_m_{0.0};
  double lidar_mount_roll_rad_{0.0};
  double lidar_mount_pitch_rad_{0.0};
  double lidar_mount_yaw_rad_{0.0};
  bool use_full_lidar_extrinsic_{false};
  Point3 lidar_translation_body_frd_m_{};
  std::array<double, 4> lidar_flu_to_body_frd_quaternion_{0.0, 1.0, 0.0, 0.0};
  double min_projected_lidar_altitude_m_{0.0};
  double max_projected_lidar_altitude_m_{100000.0};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t max_current_lidar_staleness_ns_{750'000'000};
  std::int64_t last_pose_update_ns_{0};
  std::int64_t last_scan_update_ns_{0};
  std::int64_t last_memory_snapshot_receive_ns_{0};
  std::int64_t last_memory_snapshot_stamp_ns_{0};
  std::int64_t last_memory_snapshot_diagnostic_ns_{0};
  int memory_occupied_value_{100};
  int memory_free_value_{0};
  std::size_t last_logged_path_size_{std::numeric_limits<std::size_t>::max()};
  std::size_t static_map_rectangles_{0U};
  std::size_t static_map_occupied_cells_{0U};
  std::size_t known_passage_structures_{0U};
  std::size_t known_passage_openings_{0U};
  std::uint64_t astar_runs_{0U};
  std::uint64_t astar_successes_{0U};
  std::uint64_t astar_failures_{0U};
  std::uint64_t prohibited_replans_{0U};
  std::uint64_t path_publications_{0U};
  std::uint64_t non_empty_path_publications_{0U};
  std::uint64_t hold_path_publications_{0U};
  std::uint64_t computed_path_publications_{0U};
  std::uint64_t memory_snapshot_received_{0U};
  std::uint64_t memory_snapshot_applied_{0U};
  std::uint64_t memory_snapshot_rejected_{0U};
  std::uint64_t memory_snapshot_sequence_gaps_{0U};
  std::uint64_t memory_snapshot_pending_replacements_{0U};
  std::uint64_t memory_snapshot_out_of_order_{0U};
  std::uint64_t last_memory_snapshot_received_sequence_{0U};
  std::uint64_t last_memory_snapshot_applied_sequence_{0U};
  std::uint64_t last_memory_snapshot_received_producer_instance_id_{0U};
  std::uint64_t last_memory_snapshot_applied_producer_instance_id_{0U};
  std::uint64_t memory_snapshot_producer_restarts_{0U};
  std::uint64_t memory_snapshot_applied_at_last_diagnostic_{0U};
  std::uint64_t memory_snapshot_received_at_last_diagnostic_{0U};
  std::uint64_t next_path_id_{1U};
  std::uint64_t last_published_path_id_{0U};
  std::uint64_t trajectory_generation_{0U};
  Point2 last_logged_path_first_{};
  Point2 last_logged_path_last_{};
  std::vector<Point2> last_valid_path_points_;
  std::vector<LidarProjectionPose> last_scan_projection_poses_;
  std::vector<TrajectoryPointSample> last_valid_trajectory_samples_;
  std::optional<TrajectoryDeliveryDiagnostics> pending_replan_delivery_;
  std::optional<PendingMemorySnapshot> pending_memory_snapshot_;
  double planning_duration_estimate_s_{1.0};

  mutable std::mutex memory_snapshot_mutex_;
  mutable std::mutex navigation_state_mutex_;
  mutable std::mutex lidar_input_mutex_;
  mutable std::mutex lidar_pose_history_mutex_;
  mutable std::mutex planning_request_mutex_;
  std::condition_variable_any planning_request_cv_;
  NavigationStateSnapshot live_navigation_state_{};
  LidarInputSnapshot live_lidar_input_;
  std::jthread planning_worker_;
  std::uint64_t latest_planning_request_generation_{0U};
  bool planning_request_pending_{false};

  rclcpp::CallbackGroup::SharedPtr pose_callback_group_;
  rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
  rclcpp::CallbackGroup::SharedPtr planner_control_callback_group_;
  rclcpp::CallbackGroup::SharedPtr memory_snapshot_callback_group_;
  rclcpp::Subscription<msg::ObstacleMemorySnapshot>::SharedPtr memory_snapshot_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_status_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr prohibited_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_map_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      static_building_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      known_passage_markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr path_id_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_diagnostics_pub_;
  rclcpp::Publisher<msg::ReplanBlockerEvent>::SharedPtr replan_blocker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::TimerBase::SharedPtr static_map_debug_timer_;
  rclcpp::TimerBase::SharedPtr known_passage_debug_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav
