#pragma once

#include "drone_city_nav/corridor_samples_io.hpp"
#include "drone_city_nav/current_lidar_overlay.hpp"
#include "drone_city_nav/grid_overlay.hpp"
#include "drone_city_nav/known_passage_debug_markers.hpp"
#include "drone_city_nav/known_passage_map.hpp"
#include "drone_city_nav/known_static_lidar_hit_classifier.hpp"
#include "drone_city_nav/lidar_motion_compensation.hpp"
#include "drone_city_nav/lidar_projection.hpp"
#include "drone_city_nav/navigation_pose.hpp"
#include "drone_city_nav/obstacle_memory_provenance_ros.hpp"
#include "drone_city_nav/path_smoothing.hpp"
#include "drone_city_nav/planner_core.hpp"
#include "drone_city_nav/planner_diagnostics_format.hpp"
#include "drone_city_nav/planner_node_config.hpp"
#include "drone_city_nav/planner_path_publication.hpp"
#include "drone_city_nav/planner_runtime_state.hpp"
#include "drone_city_nav/planning_grid_builder.hpp"
#include "drone_city_nav/ros_conversions.hpp"
#include "drone_city_nav/static_map_debug.hpp"
#include "drone_city_nav/static_map_source.hpp"
#include "drone_city_nav/trajectory_diagnostics_io.hpp"
#include "drone_city_nav/trajectory_planner.hpp"
#include "drone_city_nav/trajectory_refinement_scheduler.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
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
#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <limits>
#include <numbers>
#include <optional>
#include <span>
#include <sstream>
#include <string>
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

private:
  struct PendingTrajectoryRefinement {
    std::uint64_t generation{0U};
    std::uint64_t baseline_path_id{0U};
    Point2 route_start{};
    Point2 goal{};
    double baseline_length_m{std::numeric_limits<double>::quiet_NaN()};
    std::vector<Point2> route_points;
    std::string source_label;
    std::future<TrajectoryPlannerResult> future;
  };

  struct TrajectoryRefinementRequest {
    std::uint64_t generation{0U};
    std::uint64_t baseline_path_id{0U};
    Point2 route_start{};
    Point2 goal{};
    double baseline_length_m{std::numeric_limits<double>::quiet_NaN()};
    std::vector<Point2> route_points;
    std::string source_label;
    OccupancyGrid2D grid;
    std::optional<ClearanceField2D> prohibited_clearance_field;
    bool prohibited_clearance_field_cache_hit{false};
    std::vector<CorridorSample> corridor_samples;
    CorridorStats corridor_stats{};
    std::optional<KnownPassageMap> known_passages;
    TrajectoryPlannerConfig config;
  };

  void applyConfig(const PlannerNodeConfig& config);

  void onLocalPosition(const px4_msgs::msg::VehicleLocalPosition& msg);

  [[nodiscard]] bool applyMemoryGrid(const nav_msgs::msg::OccupancyGrid& msg);

  void onMemorySnapshot(const msg::ObstacleMemorySnapshot& message);

  void onScan(const sensor_msgs::msg::LaserScan& msg);

  void onAttitude(const px4_msgs::msg::VehicleAttitude& msg);

  [[nodiscard]] std::filesystem::path staticMapPackageShareDirectory() const;

  void loadConfiguredStaticMap();

  void loadConfiguredKnownPassages();

  [[nodiscard]] PlanningGridBuilderConfig planningGridBuilderConfig() const;

  [[nodiscard]] std::optional<PlanningGridBuildResult>
  buildPlanningGrid(const std::int64_t now_ns);

  void checkCurrentPathAndPublish();

  [[nodiscard]] AStarConfig astarConfigForCurrentVelocity() const;

  [[nodiscard]] static bool
  initialHeadingBiasActive(const AStarConfig& config) noexcept;

  [[nodiscard]] TrajectoryPlannerConfig
  trajectoryPlannerConfigForCurrentAltitude() const;

  [[nodiscard]] std::optional<PathComputationResult>
  computePathOnGrid(const OccupancyGrid2D& grid, const char* source_label,
                    const AStarConfig& astar_config);

  bool publishPathFromPathCells(const OccupancyGrid2D& route_grid,
                                const OccupancyGrid2D& runtime_grid,
                                const std::vector<GridIndex>& raw_cells,
                                const std::vector<GridIndex>& smoothed_cells,
                                const char* source_label,
                                const ClearanceField2D* route_clearance_field,
                                bool route_clearance_field_cache_hit);

  bool publishTrajectoryResult(const OccupancyGrid2D& validation_grid,
                               const TrajectoryPlannerResult& trajectory_result,
                               std::span<const Point2> route_points,
                               const char* source_label, double duration_ms,
                               std::uint64_t* published_path_id = nullptr);

  [[nodiscard]] bool
  keepCurrentPathAfterInvalidReplacement(const char* source_label,
                                         const char* invalid_reason) const;

  void startAsyncTrajectoryRefinement(
      const OccupancyGrid2D& grid, std::span<const Point2> route_points,
      std::uint64_t generation, std::uint64_t baseline_path_id,
      const TrajectoryPlannerResult& baseline, const char* source_label,
      const ClearanceField2D* prohibited_clearance_field,
      bool prohibited_clearance_field_cache_hit, const TrajectoryPlannerConfig& config);

  void launchScheduledTrajectoryRefinement(TrajectoryRefinementRequest request);

  void
  launchQueuedTrajectoryRefinement(std::optional<TrajectoryRefinementJob> expected_job);

  [[nodiscard]] bool
  pollPendingTrajectoryRefinement(const OccupancyGrid2D& validation_grid);

  [[nodiscard]] PublishedPathSafetySummary
  summarizePublishedPathSafety(const OccupancyGrid2D& grid,
                               const std::span<const Point2> path_points) const;

  void logPublishedPathSafety(const OccupancyGrid2D& grid,
                              const std::span<const Point2> path_points,
                              const char* source_label) const;

  [[nodiscard]] bool connectRouteToCurrentPose(const OccupancyGrid2D& grid,
                                               std::vector<Point2>& path_points,
                                               const char* source_label) const;

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
                        const TrajectoryPlannerStats* trajectory_stats = nullptr);

  void publishTrajectoryDiagnostics(const std::uint64_t path_id,
                                    const std::uint64_t path_stamp_ns,
                                    const TrajectoryPlannerStats& stats) const;

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
  double known_static_lidar_hit_closer_range_tolerance_m_{0.5};
  double known_static_lidar_hit_farther_range_tolerance_m_{1.5};
  double current_altitude_m_{std::numeric_limits<double>::quiet_NaN()};
  double current_speed_mps_{std::numeric_limits<double>::quiet_NaN()};
  double last_scan_pose_lag_s_{0.0};
  double last_scan_pose_latency_s_{0.0};
  double last_scan_motion_shift_m_{0.0};
  double lidar_pose_latency_s_{0.05};
  Point2 last_scan_motion_shift_{};
  double lidar_z_offset_m_{0.0};
  double lidar_mount_roll_rad_{0.0};
  double lidar_mount_pitch_rad_{0.0};
  double lidar_mount_yaw_rad_{0.0};
  double min_projected_lidar_altitude_m_{0.0};
  double max_projected_lidar_altitude_m_{100000.0};
  std::int64_t max_pose_staleness_ns_{1'000'000'000};
  std::int64_t max_current_lidar_staleness_ns_{750'000'000};
  std::int64_t last_pose_update_ns_{0};
  std::int64_t last_scan_update_ns_{0};
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
  std::uint64_t next_path_id_{1U};
  std::uint64_t last_published_path_id_{0U};
  std::uint64_t trajectory_generation_{0U};
  Point2 last_logged_path_first_{};
  Point2 last_logged_path_last_{};
  std::vector<Point2> last_valid_path_points_;
  std::vector<TrajectoryPointSample> last_valid_trajectory_samples_;
  std::optional<PendingTrajectoryRefinement> pending_refinement_;
  std::optional<TrajectoryRefinementRequest> queued_refinement_;
  TrajectoryRefinementScheduler refinement_scheduler_;

  rclcpp::Subscription<msg::ObstacleMemorySnapshot>::SharedPtr memory_snapshot_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_pub_;
  rclcpp::TimerBase::SharedPtr static_map_debug_timer_;
  rclcpp::TimerBase::SharedPtr known_passage_debug_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace drone_city_nav
