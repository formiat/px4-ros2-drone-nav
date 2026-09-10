// Offline probe of the persistent planner on a recorded raw snapshot: loads
// the grid a flight captured, runs the planner from a start to the mission
// goal with the production configuration, and prints what each update finds.
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/occupied_collision_oracle_3d.hpp"
#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "../src/runtime/ros/production_mppi_config_ros.hpp"

namespace {

std::shared_ptr<const drone_city_nav::ObservedOccupancyGrid3D>
loadGrid(const std::string& path) {
  std::ifstream input{path};
  drone_city_nav::GridBounds3D bounds;
  int chunk_size{0};
  std::size_t chunk_count{0U};
  input >> bounds.origin_x >> bounds.origin_y >> bounds.origin_z >>
      bounds.resolution_m >> bounds.width_cells >> bounds.height_cells >>
      bounds.depth_cells >> chunk_size >> chunk_count;
  if (!input || chunk_size != drone_city_nav::ObservedOccupancyGrid3D::kChunkSize) {
    std::fprintf(stderr, "bad grid header (chunk size %d)\n", chunk_size);
    std::exit(2);
  }
  auto grid = std::make_shared<drone_city_nav::ObservedOccupancyGrid3D>(bounds);
  for (std::size_t chunk = 0U; chunk < chunk_count; ++chunk) {
    drone_city_nav::OccupancyChunkIndex3D index{};
    input >> index.x >> index.y >> index.z;
    drone_city_nav::ObservedOccupancyChunk3D words{};
    for (auto& word : words.observed) {
      std::string hex;
      input >> hex;
      word = std::strtoull(hex.c_str(), nullptr, 16);
    }
    for (auto& word : words.occupied) {
      std::string hex;
      input >> hex;
      word = std::strtoull(hex.c_str(), nullptr, 16);
    }
    static_cast<void>(grid->replaceChunk(index, words));
  }
  return grid;
}

} // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() < 9U) {
    std::fprintf(stderr,
                 "usage: persistent_planner_probe GRID sx sy sz gx gy gz UPDATES "
                 "--ros-args --params-file urban_mvp.yaml\n");
    return 2;
  }
  auto node = std::make_shared<rclcpp::Node>("production_mppi_node");
  const drone_city_nav::ProductionMppiConfig config =
      drone_city_nav::declareProductionMppiConfig(*node);
  drone_city_nav::PersistentDStarLitePlanner3D planner{
      config.planning.persistent_planner};
  const auto grid = loadGrid(args[1]);
  drone_city_nav::PersistentPlannerWorld3D world{
      .observed_occupancy = grid,
      .static_occupancy = nullptr,
      .producer_instance_id = 1U,
      .revision = 1U,
      .occupied_fingerprint = grid->occupiedContentFingerprint(),
      .full_reset = true,
  };
  drone_city_nav::PersistentPlannerRequest3D request{
      .start = {std::stod(args[2]), std::stod(args[3]), std::stod(args[4])},
      .mission_goal = {std::stod(args[5]), std::stod(args[6]), std::stod(args[7])},
      .mission_epoch = 1U,
      .world = world,
      .session_id = 1U,
  };
  const int updates = std::stoi(args[8]);
  if (updates < 0) {
    // Segment mode: validate straight segments with the planner's body, the
    // way the lattice sweeps its edges. Segments follow as x y z x y z ...
    drone_city_nav::OccupiedCollisionOracle3D oracle{
        drone_city_nav::OccupiedCollisionWorld3D{
            .observed_occupancy = grid.get(),
            .footprint = config.planning.persistent_planner.physical_footprint,
            .flight_envelope = config.planning.persistent_planner.flight_envelope,
        }};
    for (std::size_t i = 9U; i + 5U < args.size(); i += 6U) {
      const drone_city_nav::Point3 a{std::stod(args[i]), std::stod(args[i + 1]),
                                     std::stod(args[i + 2])};
      const drone_city_nav::Point3 b{std::stod(args[i + 3]), std::stod(args[i + 4]),
                                     std::stod(args[i + 5])};
      const auto r = oracle.validateSegment(a, drone_city_nav::FootprintBodyAxis{}, b,
                                            drone_city_nav::FootprintBodyAxis{});
      std::printf("segment (%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f): clear=%d "
                  "failure=(%.2f,%.2f,%.2f)\n",
                  a.x, a.y, a.z, b.x, b.y, b.z, r.clear() ? 1 : 0, r.failure_point.x,
                  r.failure_point.y, r.failure_point.z);
    }
    rclcpp::shutdown();
    return 0;
  }
  std::printf("grid known=%zu free=%zu occupied=%zu\n", grid->knownVoxelCount(),
              grid->freeVoxelCount(), grid->occupiedVoxelCount());
  for (int update = 0; update < updates; ++update) {
    const drone_city_nav::PlannerUpdate3D result = planner.plan(request);
    const auto& t = result.telemetry;
    std::printf("update %d input=%s progress=%s publishable=%d feas=%d/%d fexp=%zu "
                "exhausted=%d "
                "closed=%d closest=%.1f restarts=%zu records=%zu texp=%zu tobj=%.1f "
                "search_ms=%.1f\n",
                update, t.input_failure,
                drone_city_nav::searchProgress3DName(result.progress),
                result.publishable() ? 1 : 0, t.feasibility_attempted ? 1 : 0,
                t.feasibility_route_found ? 1 : 0, t.feasibility_expansions,
                t.feasibility_frontier_exhausted ? 1 : 0,
                t.feasibility_anchor_in_closed_component ? 1 : 0,
                t.feasibility_closest_goal_distance_m, t.feasibility_restarts,
                t.records, t.execution_time_search_expansions,
                t.execution_time_search_objective_s, t.search_ms);
    if (result.improved_incumbent.has_value()) {
      const auto& c = *result.improved_incumbent;
      std::printf("  candidate source=%s length=%.1f eta=%.1f ranked=%.1f points=%zu:",
                  drone_city_nav::spatialRouteCandidateSource3DName(c.source),
                  c.path_length_m, c.estimated_execution_time_s,
                  c.ranked_execution_time_s, c.points.size());
      for (const auto& p : c.points) {
        std::printf(" (%.1f,%.1f,%.1f)", p.x, p.y, p.z);
      }
      std::printf("\n");
    }
  }
  rclcpp::shutdown();
  return 0;
}
