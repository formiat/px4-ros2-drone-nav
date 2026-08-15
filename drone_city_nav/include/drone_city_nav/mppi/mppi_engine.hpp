#pragma once

#include "drone_city_nav/mppi/mppi_config.hpp"
#include "drone_city_nav/mppi/mppi_post_update_classification.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <vector>

namespace drone_city_nav::mppi {

struct EsdfSnapshot {
  EsdfGrid grid{};
  std::span<const float> distances_m;
  std::uint64_t revision{0U};
};

struct KnownSolid {
  float center_x_m{0.0F};
  float center_y_m{0.0F};
  float normal_x{1.0F};
  float normal_y{0.0F};
  float lateral_x{0.0F};
  float lateral_y{1.0F};
  float half_depth_m{0.0F};
  float half_width_m{0.0F};
  float min_z_m{0.0F};
  float max_z_m{0.0F};
};

struct RouteSample3D {
  float x_m{0.0F};
  float y_m{0.0F};
  float z_m{0.0F};
  float tangent_x{0.0F};
  float tangent_y{0.0F};
  float tangent_z{0.0F};
  float station_m{0.0F};
  float reference_speed_mps{0.0F};
  RiskTier required_risk_tier{RiskTier::kPreferred};
};

struct RouteReference {
  std::shared_ptr<const std::vector<RouteSample3D>> points;
  std::uint64_t generation{0U};
  float initial_station_m{0.0F};
};

enum class DeterministicCandidateKind : std::uint8_t {
  kDisabled,
  kTargetDirectedReacquisition,
  kRouteDirectedCruise,
};

struct MppiTickInput {
  State initial_state{};
  State target{};
  std::uint64_t pose_revision{0U};
  std::uint64_t obstacle_revision{0U};
  std::int64_t planning_stamp_ns{0};
  std::optional<Control> previous_applied_control;
  std::uint64_t nominal_reseed_generation{0U};
  float reference_speed_mps{-1.0F};
  std::optional<MovingTargetReference> moving_target;
  std::optional<RouteReference> route;
  std::vector<DynamicAircraftTrajectory> dynamic_aircraft;
  std::optional<DynamicAircraftCostPolicy> dynamic_aircraft_cost_policy;
  std::optional<CooperativeManeuverPreference> cooperative_maneuver;
  std::optional<CooperativeSeparationAcquisition> cooperative_acquisition;
  std::optional<NonCooperativeSeparationAcquisition> noncooperative_acquisition;
  std::optional<std::size_t> active_rollouts;
  DeterministicCandidateKind deterministic_candidate{
      DeterministicCandidateKind::kDisabled};
  bool cooperative_avoidance_active{false};
  bool noncooperative_avoidance_active{false};
};

[[nodiscard]] inline std::size_t
resolveMppiActiveRollouts(const std::size_t capacity,
                          const std::optional<std::size_t> requested) {
  const std::size_t active = requested.value_or(capacity);
  if (capacity == 0U || active == 0U || active > capacity) {
    throw std::invalid_argument{"active MPPI rollout count exceeds capacity"};
  }
  return active;
}

struct MppiStageTimings {
  double noise_generation_ms{0.0};
  double rollout_simulation_ms{0.0};
  double risk_reduction_ms{0.0};
  double weight_calculation_ms{0.0};
  double control_update_ms{0.0};
  double warm_start_ms{0.0};
  double gpu_total_ms{0.0};
  double repair_validation_ms{0.0};
  double post_update_evaluation_ms{0.0};
  double host_total_ms{0.0};
  double horizon_reconstruction_ms{0.0};
};

enum class MppiPostUpdateRepair : std::uint8_t {
  kNotRequired,
  kBacktracked,
  kBestFeasibleRollout,
  kDeterministicCandidate,
  kFailed,
};

[[nodiscard]] inline const char*
mppiPostUpdateRepairName(const MppiPostUpdateRepair repair) noexcept {
  switch (repair) {
    case MppiPostUpdateRepair::kNotRequired:
      return "not_required";
    case MppiPostUpdateRepair::kBacktracked:
      return "backtracked";
    case MppiPostUpdateRepair::kBestFeasibleRollout:
      return "best_feasible_rollout";
    case MppiPostUpdateRepair::kDeterministicCandidate:
      return "deterministic_candidate";
    case MppiPostUpdateRepair::kFailed:
      return "failed";
  }
  return "unknown";
}

struct MppiTickResult {
  std::vector<State> horizon;
  std::vector<Control> controls;
  MppiFeasibilityContract feasibility_contract{};
  MppiPostUpdateClassificationResult post_update_classification{};
  MppiPostUpdateRepair post_update_repair{MppiPostUpdateRepair::kNotRequired};
  float post_update_backtrack_ratio{1.0F};
  RiskTier selected_tier{RiskTier::kCollision};
  bool altitude_envelope_violation{false};
  bool raw_collision{true};
  bool known_solid_collision{false};
  float critical_exposure_m{0.0F};
  float planning_exposure_m{0.0F};
  float critical_clearance_proximity_s{0.0F};
  float obstacle_approach_m2_s{0.0F};
  float minimum_esdf_distance_m{0.0F};
  float head_progress_m{0.0F};
  float terminal_progress_m{0.0F};
  float minimum_target_separation_m{0.0F};
  float minimum_peer_separation_m{0.0F};
  float peer_separation_cost{0.0F};
  float dynamic_aircraft_anticipation_cost{0.0F};
  float dynamic_aircraft_survival_cost{0.0F};
  float dynamic_aircraft_survival_cost_ratio{0.0F};
  float predicted_capture_time_s{-1.0F};
  float maximum_acceleration_mps2{0.0F};
  float maximum_jerk_mps3{0.0F};
  float first_control_delta{0.0F};
  double warm_start_shift_s{0.0};
  bool nominal_reseeded{false};
  bool target_directed_candidate_injected{false};
  bool target_directed_candidate_raw_safe{false};
  bool target_directed_candidate_best_feasible{false};
  float target_directed_candidate_weight{0.0F};
  bool route_directed_candidate_injected{false};
  bool route_directed_candidate_raw_safe{false};
  bool route_directed_candidate_best_feasible{false};
  float route_directed_candidate_weight{0.0F};
  std::uint64_t route_directed_candidate_generation{0U};
  bool cooperative_acquisition_reseeded{false};
  bool cooperative_release_reseeded{false};
  bool cooperative_acquisition_available{false};
  bool cooperative_acquisition_positive_progress{false};
  bool cooperative_acquisition_backward_fallback{false};
  std::size_t cooperative_acquisition_candidate_index{0U};
  float cooperative_acquisition_head_progress_m{0.0F};
  float cooperative_acquisition_terminal_progress_m{0.0F};
  float cooperative_acquisition_separation_gain_m{0.0F};
  bool cooperative_candidates_injected{false};
  bool noncooperative_acquisition_reseeded{false};
  bool noncooperative_release_reseeded{false};
  bool noncooperative_acquisition_available{false};
  std::size_t noncooperative_acquisition_candidate_index{0U};
  NonCooperativeManeuver noncooperative_acquisition_maneuver{
      NonCooperativeManeuver::kRouteCruise};
  float noncooperative_acquisition_minimum_separation_m{0.0F};
  float noncooperative_acquisition_separation_gain_m{0.0F};
  float noncooperative_acquisition_head_progress_m{0.0F};
  float noncooperative_acquisition_terminal_progress_m{0.0F};
  std::size_t dynamic_aircraft_count{0U};
  std::uint64_t esdf_revision{0U};
  std::size_t active_rollouts{0U};
  MppiStageTimings timings{};
};

struct EsdfUploadResult {
  bool accepted{false};
  double upload_ms{0.0};
  std::uint64_t revision{0U};
};

class MppiCudaEngine {
public:
  explicit MppiCudaEngine(BenchmarkConfig config);
  ~MppiCudaEngine();

  MppiCudaEngine(const MppiCudaEngine&) = delete;
  MppiCudaEngine& operator=(const MppiCudaEngine&) = delete;
  MppiCudaEngine(MppiCudaEngine&&) noexcept;
  MppiCudaEngine& operator=(MppiCudaEngine&&) noexcept;

  [[nodiscard]] EsdfUploadResult updateEsdf(const EsdfSnapshot& snapshot);
  void updateKnownSolids(std::span<const KnownSolid> solids);
  [[nodiscard]] MppiTickResult plan(const MppiTickInput& input);
  [[nodiscard]] std::size_t allocatedBytes() const noexcept;
  [[nodiscard]] bool ready() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace drone_city_nav::mppi
