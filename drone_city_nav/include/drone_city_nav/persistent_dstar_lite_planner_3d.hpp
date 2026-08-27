#pragma once

#include "drone_city_nav/flight_envelope.hpp"
#include "drone_city_nav/flight_time_model_3d.hpp"
#include "drone_city_nav/observed_occupancy_grid_3d.hpp"
#include "drone_city_nav/swept_footprint.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace drone_city_nav {

enum class PersistentPlannerStatus3D : std::uint8_t {
  kInvalidInput,
  kReachedMissionGoal,
  kSearchInProgress,
  kNoRoute,
  kStartUnavailable,
  kGoalUnavailable,
};

struct PersistentPlannerWorld3D {
  std::shared_ptr<const ObservedOccupancyGrid3D> observed_occupancy;
  std::shared_ptr<const OccupancyGrid3D> static_occupancy;
  std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed;
  std::optional<LaunchSupportContact3D> launch_support_contact;
  std::vector<OccupancyChunkIndex3D> dirty_chunks;
  std::uint64_t producer_instance_id{0U};
  std::uint64_t revision{0U};
  std::uint64_t occupied_fingerprint{0U};
  bool full_reset{false};

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] const GridBounds3D* bounds() const noexcept;
};

struct PersistentPlannerConfig3D {
  double horizontal_step_m{2.0};
  double vertical_step_m{1.0};
  FlightTimeModel3D time_model{};
  double minimum_continuous_turn_alignment{0.7071067811865476};
  double goal_tolerance_m{1.0};
  std::size_t connector_search_radius_cells{2U};
  std::size_t maximum_expansions_per_update{200000U};
  std::size_t maximum_incremental_changed_voxels{32768U};
  std::size_t maximum_extracted_path_nodes{8192U};
  std::size_t maximum_shortcut_checks{8192U};
  double maximum_compute_time_ms{150.0};
  SweptFootprintConfig physical_footprint{};
  FlightEnvelopeConfig flight_envelope{};
};

struct PersistentPlannerRequest3D {
  Point3 start{};
  Vec3 velocity{};
  Point3 mission_goal{};
  std::uint64_t mission_epoch{0U};
  PersistentPlannerWorld3D world{};
};

struct PersistentPlannerResult3D {
  PersistentPlannerStatus3D status{PersistentPlannerStatus3D::kInvalidInput};
  std::vector<Point3> points;
  std::uint64_t mission_epoch{0U};
  std::uint64_t planned_on_revision{0U};
  std::uint64_t occupied_fingerprint{0U};
  std::uint64_t search_generation{0U};
  std::uint64_t repair_generation{0U};
  std::size_t expansions{0U};
  std::size_t changed_occupied_voxels{0U};
  std::size_t affected_lattice_states{0U};
  std::size_t records{0U};
  std::size_t open_entries{0U};
  std::size_t shortcut_checks{0U};
  std::size_t shortcuts_applied{0U};
  double path_length_m{0.0};
  double estimated_execution_time_s{0.0};
  double estimated_translation_time_s{0.0};
  double estimated_stationary_turn_time_s{0.0};
  double world_update_ms{0.0};
  double search_ms{0.0};
  bool search_state_reused{false};
  bool occupied_world_unchanged{false};
  bool incumbent_retained{false};
  bool search_complete{false};

  [[nodiscard]] bool executable() const noexcept;
};

namespace detail {
class PersistentDStarLitePlanner3DImpl;
}

class PersistentDStarLitePlanner3D final {
public:
  explicit PersistentDStarLitePlanner3D(PersistentPlannerConfig3D config = {});
  ~PersistentDStarLitePlanner3D();

  PersistentDStarLitePlanner3D(const PersistentDStarLitePlanner3D&) = delete;
  PersistentDStarLitePlanner3D& operator=(const PersistentDStarLitePlanner3D&) = delete;
  PersistentDStarLitePlanner3D(PersistentDStarLitePlanner3D&&) noexcept;
  PersistentDStarLitePlanner3D& operator=(PersistentDStarLitePlanner3D&&) noexcept;

  [[nodiscard]] PersistentPlannerResult3D
  plan(const PersistentPlannerRequest3D& request);
  void reset() noexcept;
  [[nodiscard]] const PersistentPlannerConfig3D& config() const noexcept;

private:
  std::unique_ptr<detail::PersistentDStarLitePlanner3DImpl> implementation_;
};

[[nodiscard]] const char*
persistentPlannerStatus3DName(PersistentPlannerStatus3D status) noexcept;

} // namespace drone_city_nav
