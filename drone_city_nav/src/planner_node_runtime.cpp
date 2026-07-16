#include <array>
#include <optional>
#include <ranges>

#include "planner_node.hpp"

namespace drone_city_nav {
namespace {

struct RawSourceProbe {
  const char* name{""};
  const OccupancyGrid2D* grid{nullptr};
};

struct NearestRawSource {
  const char* name{""};
  GridIndex cell{};
  Point2 center{};
  double distance_m{std::numeric_limits<double>::infinity()};
};

[[nodiscard]] bool sourceGridMatches(const OccupancyGrid2D& planning_grid,
                                     const OccupancyGrid2D* const source_grid) {
  return source_grid != nullptr && haveSameGridGeometry(planning_grid, *source_grid);
}

[[nodiscard]] std::string joinSourceNames(const std::vector<const char*>& names) {
  if (names.empty()) {
    return "none";
  }

  std::ostringstream stream;
  for (std::size_t index = 0U; index < names.size(); ++index) {
    if (index > 0U) {
      stream << ",";
    }
    stream << names[index];
  }
  return stream.str();
}

[[nodiscard]] std::vector<const char*>
exactRawSources(const std::array<RawSourceProbe, 3U>& sources,
                const OccupancyGrid2D& planning_grid, const GridIndex cell) {
  std::vector<const char*> exact_sources;
  for (const RawSourceProbe& source : sources) {
    if (!sourceGridMatches(planning_grid, source.grid) ||
        !source.grid->isOccupied(cell)) {
      continue;
    }
    exact_sources.push_back(source.name);
  }
  return exact_sources;
}

[[nodiscard]] std::optional<NearestRawSource>
nearestRawSource(const std::array<RawSourceProbe, 3U>& sources,
                 const OccupancyGrid2D& planning_grid, const GridIndex query_cell,
                 const double search_radius_m) {
  if (!(planning_grid.resolution() > 0.0) || !(search_radius_m >= 0.0)) {
    return std::nullopt;
  }

  const Point2 query_center = planning_grid.cellCenter(query_cell);
  const int radius_cells =
      static_cast<int>(std::ceil(search_radius_m / planning_grid.resolution()));
  const double search_radius_sq = search_radius_m * search_radius_m;
  std::optional<NearestRawSource> nearest;
  for (const RawSourceProbe& source : sources) {
    if (!sourceGridMatches(planning_grid, source.grid)) {
      continue;
    }
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const GridIndex candidate{query_cell.x + dx, query_cell.y + dy};
        if (!planning_grid.contains(candidate) || !source.grid->isOccupied(candidate)) {
          continue;
        }
        const Point2 candidate_center = planning_grid.cellCenter(candidate);
        const double distance_sq = squaredDistance(query_center, candidate_center);
        if (distance_sq > search_radius_sq) {
          continue;
        }
        const double distance_m = std::sqrt(distance_sq);
        if (!nearest.has_value() || distance_m < nearest->distance_m) {
          nearest =
              NearestRawSource{source.name, candidate, candidate_center, distance_m};
        }
      }
    }
  }

  return nearest;
}

[[nodiscard]] const KnownStaticLidarHitProvenance*
findCurrentLidarProvenance(const CurrentLidarOverlayStats& stats,
                           const GridIndex cell) {
  const auto iter = std::ranges::find_if(
      stats.retained_known_static_hits,
      [cell](const KnownStaticLidarHitProvenance& provenance) {
        return provenance.cell_x == cell.x && provenance.cell_y == cell.y;
      });
  return iter == stats.retained_known_static_hits.end() ? nullptr : &*iter;
}

} // namespace

void PlannerNode::invalidateCurrentPose() {
  current_pose_ = Pose2{};
  current_altitude_m_ = std::numeric_limits<double>::quiet_NaN();
  pose_valid_ = false;
  altitude_valid_ = false;
  last_scan_projection_pose_valid_ = false;
  last_pose_update_ns_ = 0;
}

[[nodiscard]] double PlannerNode::poseAgeSeconds(const std::int64_t now_ns) const {
  return ageSecondsFromStamp(last_pose_update_ns_, now_ns);
}

[[nodiscard]] double PlannerNode::scanAgeSeconds(const std::int64_t now_ns) const {
  return ageSecondsFromStamp(last_scan_update_ns_, now_ns);
}

std::string PlannerNode::describeProhibitedIntersectionSource(
    const OccupancyGrid2D& grid, const PathProhibitedIntersection& intersection,
    const PlanningGridBuildResult& planning_result) {
  const OccupancyGrid2D* memory_source_grid = nullptr;
  if (memory_grid_) {
    memory_source_grid = &*memory_grid_;
  }
  const std::array<RawSourceProbe, 3U> sources{
      RawSourceProbe{"static", static_grid_ ? &*static_grid_ : nullptr},
      RawSourceProbe{"memory", memory_source_grid},
      RawSourceProbe{"current_lidar", planning_result.current_lidar_grid
                                          ? &*planning_result.current_lidar_grid
                                          : nullptr}};

  const std::vector<const char*> exact_sources =
      exactRawSources(sources, grid, intersection.cell);
  const double search_radius_m = inflation_radius_m_ + std::max(0.0, grid.resolution());
  const std::optional<NearestRawSource> nearest_source =
      nearestRawSource(sources, grid, intersection.cell, search_radius_m);

  std::ostringstream stream;
  stream << "raw_sources[exact=" << joinSourceNames(exact_sources);
  if (nearest_source.has_value()) {
    stream << " nearest=" << nearest_source->name
           << " nearest_distance=" << nearest_source->distance_m << "m"
           << " nearest_cell=(" << nearest_source->cell.x << ", "
           << nearest_source->cell.y << ")" << " nearest_center=("
           << nearest_source->center.x << ", " << nearest_source->center.y << ")";
  } else {
    stream << " nearest=none nearest_distance=nanm nearest_cell=(-1, -1)"
           << " nearest_center=(nan, nan)";
  }
  stream << " search_radius=" << search_radius_m << "m"
         << " static_used=" << (planning_result.static_source.used ? "true" : "false")
         << " memory_used=" << (planning_result.memory.used ? "true" : "false")
         << " current_lidar_used="
         << (planning_result.current_lidar.used ? "true" : "false")
         << " current_lidar_fresh="
         << (planning_result.current_lidar.fresh ? "true" : "false") << "]";
  std::optional<GridIndex> current_lidar_provenance_cell;
  const bool exact_current_lidar = std::any_of(
      exact_sources.begin(), exact_sources.end(), [](const char* source_name) {
        return std::string_view{source_name} == "current_lidar";
      });
  if (exact_current_lidar && planning_result.current_lidar_grid.has_value()) {
    current_lidar_provenance_cell = planning_result.current_lidar_grid->worldToCell(
        grid.cellCenter(intersection.cell));
  } else if (nearest_source.has_value() &&
             std::string_view{nearest_source->name} == "current_lidar") {
    current_lidar_provenance_cell = nearest_source->cell;
  }
  if (current_lidar_provenance_cell.has_value()) {
    stream << ' '
           << formatCurrentLidarAcceptedHitDiagnostic(planning_result.current_lidar,
                                                      *current_lidar_provenance_cell);
    if (const KnownStaticLidarHitProvenance* provenance = findCurrentLidarProvenance(
            planning_result.current_lidar, *current_lidar_provenance_cell);
        provenance != nullptr) {
      stream << " known_static_hit[classification="
             << knownStaticLidarHitClassificationName(provenance->classification)
             << " structure=" << provenance->structure_id
             << " opening=" << provenance->opening_id << " part=" << provenance->part_id
             << " cell=(" << provenance->cell_x << ", " << provenance->cell_y
             << ") endpoint=(" << provenance->endpoint_map_m.x << ", "
             << provenance->endpoint_map_m.y << ", " << provenance->endpoint_map_m.z
             << ") measured_range=" << provenance->measured_range_m
             << " expected_range=" << provenance->expected_range_m
             << " delta=" << provenance->range_delta_m << "]";
    } else {
      stream << " known_static_hit[unavailable]";
    }
  }
  std::optional<GridIndex> memory_provenance_cell;
  const bool exact_memory = std::any_of(
      exact_sources.begin(), exact_sources.end(), [](const char* source_name) {
        return std::string_view{source_name} == "memory";
      });
  if (exact_memory && memory_grid_.has_value()) {
    memory_provenance_cell =
        memory_grid_->worldToCell(grid.cellCenter(intersection.cell));
  } else if (nearest_source.has_value() &&
             std::string_view{nearest_source->name} == "memory") {
    memory_provenance_cell = nearest_source->cell;
  }

  const MemoryProvenanceMatchResult provenance_match =
      memory_provenance_snapshot_.has_value()
          ? MemoryProvenanceMatchResult{&*memory_provenance_snapshot_,
                                        MemoryProvenanceUnavailableReason::kNone}
          : MemoryProvenanceMatchResult{};
  stream << ' ' << memorySnapshotTransportDiagnostic(get_clock()->now().nanoseconds())
         << ' '
         << formatMemoryProvenanceDiagnostic(provenance_match, memory_provenance_cell);
  return stream.str();
}

bool PlannerNode::keepCurrentPathIfStillClear(
    const OccupancyGrid2D& grid, const PlanningGridBuildResult& planning_result) {
  if (last_valid_path_points_.size() < 2U) {
    return false;
  }

  const StablePathDecision decision = planner_core_.evaluateStablePath(
      grid, last_valid_path_points_, current_pose_.position, goal_);
  if (stablePathRuntimeAction(decision.reason) == StablePathRuntimeAction::kRunAStar &&
      decision.reason != StablePathDecisionReason::kProhibitedConfirmed) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Stable path reuse rejected; running A*: reason=%s "
        "previous_waypoints=%zu endpoint_goal_distance=%.2fm "
        "goal_tolerance=%.2fm deviation=%.2fm "
        "current=(%.2f, %.2f) goal=(%.2f, %.2f)",
        stablePathDecisionReasonName(decision.reason), last_valid_path_points_.size(),
        decision.endpoint_goal_distance_m, stable_path_goal_tolerance_m_,
        decision.deviation_m, current_pose_.position.x, current_pose_.position.y,
        goal_.x, goal_.y);
    return false;
  }

  if (decision.reason == StablePathDecisionReason::kProhibitedConfirmed) {
    const Point2 prohibited_start =
        decision.prohibited_segment_index < decision.remaining_path.size()
            ? decision.remaining_path[decision.prohibited_segment_index]
            : Point2{};
    const Point2 prohibited_end =
        decision.prohibited_segment_index + 1U < decision.remaining_path.size()
            ? decision.remaining_path[decision.prohibited_segment_index + 1U]
            : Point2{};
    const PathProhibitedIntersection intersection =
        decision.prohibited_intersection.value_or(PathProhibitedIntersection{});
    const std::string source_diagnostic =
        decision.prohibited_intersection.has_value()
            ? describeProhibitedIntersectionSource(grid, intersection, planning_result)
            : std::string{"raw_sources[exact=unknown nearest=unknown "
                          "nearest_distance=nanm nearest_cell=(-1, -1) "
                          "nearest_center=(nan, nan) search_radius=nanm]"};
    ++prohibited_replans_;
    RCLCPP_WARN(get_logger(),
                "Current path intersects newly available prohibited obstacle data; "
                "running A* from current pose: reason=%s "
                "remaining_waypoints=%zu deviation=%.2fm prohibited_segment=%zu "
                "segment_start=(%.2f, %.2f) segment_end=(%.2f, %.2f) "
                "blocker[cell=(%d, %d) center=(%.2f, %.2f) occupied=%s inflated=%s "
                "line_cell=%zu/%zu segment_t=%.3f segment_distance=%.2fm "
                "path_distance=%.2fm segment_start_prohibited=%s "
                "segment_end_prohibited=%s] %s",
                stablePathDecisionReasonName(decision.reason),
                decision.remaining_path.size(), decision.deviation_m,
                decision.prohibited_segment_index, prohibited_start.x,
                prohibited_start.y, prohibited_end.x, prohibited_end.y,
                intersection.cell.x, intersection.cell.y, intersection.cell_center.x,
                intersection.cell_center.y, intersection.occupied ? "true" : "false",
                intersection.inflated ? "true" : "false", intersection.line_cell_index,
                intersection.line_cell_count, intersection.segment_t,
                intersection.segment_distance_m, intersection.path_distance_m,
                intersection.segment_start_prohibited ? "true" : "false",
                intersection.segment_end_prohibited ? "true" : "false",
                source_diagnostic.c_str());
    return false;
  }

  last_valid_path_points_ = decision.remaining_path;
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Keeping current path because the remaining path is still clear: "
      "reason=%s remaining_waypoints=%zu deviation=%.2fm distance_to_goal=%.2f",
      stablePathDecisionReasonName(decision.reason), last_valid_path_points_.size(),
      decision.deviation_m, distance(current_pose_.position, goal_));
  return true;
}

void PlannerNode::logPathUpdate(const nav_msgs::msg::Path& path,
                                const PathMetrics& metrics,
                                const PathPublicationReason reason,
                                const std::uint64_t path_id) {
  const std::size_t path_size = path.poses.size();
  const bool path_changed = path_size != last_logged_path_size_;
  const std::string counters = plannerCountersSummary();
  const std::uint64_t path_stamp_ns = stampNanoseconds(path.header.stamp);
  if (path_size == 0U) {
    if (path_changed) {
      RCLCPP_WARN(get_logger(),
                  "Published empty path: path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                  " reason=%s counters[%s]",
                  path_id, path_stamp_ns, pathPublicationReasonName(reason),
                  counters.c_str());
      last_logged_path_size_ = path_size;
    }
    return;
  }

  const Point2 first{path.poses.front().pose.position.x,
                     path.poses.front().pose.position.y};
  const Point2 last{path.poses.back().pose.position.x,
                    path.poses.back().pose.position.y};
  const bool endpoint_changed =
      squaredDistance(first, last_logged_path_first_) > 0.01 ||
      squaredDistance(last, last_logged_path_last_) > 0.01;
  if (path_changed || endpoint_changed) {
    std::vector<Point2> preview_points;
    preview_points.reserve(path.poses.size());
    for (const auto& pose : path.poses) {
      preview_points.push_back(Point2{pose.pose.position.x, pose.pose.position.y});
    }
    const std::string preview = pathPreview(preview_points, 6U);
    RCLCPP_INFO(get_logger(),
                "Published path: path_id=%" PRIu64 " path_stamp_ns=%" PRIu64
                " reason=%s waypoints=%zu segments=%zu "
                "straight_segments=%zu turns=%zu length=%.2f first=(%.2f, %.2f) "
                "segment_lengths[min=%.2f mean=%.2f max=%.2f lt2=%zu lt5=%zu "
                "lt10=%zu] "
                "last=(%.2f, %.2f) counters[%s] preview=%s",
                path_id, path_stamp_ns, pathPublicationReasonName(reason), path_size,
                metrics.segments, metrics.straight_segments, metrics.turns,
                metrics.length_m, first.x, first.y, metrics.min_segment_length_m,
                metrics.mean_segment_length_m, metrics.max_segment_length_m,
                metrics.segments_shorter_than_2m, metrics.segments_shorter_than_5m,
                metrics.segments_shorter_than_10m, last.x, last.y, counters.c_str(),
                preview.c_str());
    last_logged_path_size_ = path_size;
    last_logged_path_first_ = first;
    last_logged_path_last_ = last;
  }
}

void PlannerNode::recordPathPublication(const PathPublicationReason reason,
                                        const bool empty_path) {
  PathPublicationCounters counters{path_publications_, non_empty_path_publications_,
                                   hold_path_publications_,
                                   computed_path_publications_};
  drone_city_nav::recordPathPublication(counters, reason, empty_path);
  path_publications_ = counters.path_publications;
  non_empty_path_publications_ = counters.non_empty_path_publications;
  hold_path_publications_ = counters.hold_path_publications;
  computed_path_publications_ = counters.computed_path_publications;
}

[[nodiscard]] std::string PlannerNode::plannerCountersSummary() const {
  return drone_city_nav::plannerCountersSummary(PlannerCountersSnapshot{
      astar_runs_, astar_successes_, astar_failures_, prohibited_replans_,
      PathPublicationCounters{path_publications_, non_empty_path_publications_,
                              hold_path_publications_, computed_path_publications_}});
}

void PlannerNode::logPlannerCountersThrottled() {
  const std::string counters = plannerCountersSummary();
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Planner counters: %s",
                       counters.c_str());
}

} // namespace drone_city_nav
