#pragma once

#include "drone_city_nav/certified_route_splice_3d.hpp"
#include "drone_city_nav/compiled_trajectory_3d.hpp"
#include "drone_city_nav/dynamic_handoff_validator_3d.hpp"
#include "drone_city_nav/execution_route_store_3d.hpp"
#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"
#include "drone_city_nav/route_decoration_compiler_3d.hpp"
#include "drone_city_nav/route_lifecycle_3d.hpp"
#include "drone_city_nav/route_successor_improvement_3d.hpp"
#include "drone_city_nav/static_route_extension.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace drone_city_nav {

enum class ProductionPlanningSearchKind : std::uint8_t {
  kNone,
  kPersistentDStarLite3D,
};

struct ProductionPersistentPlannerTelemetry3D {
  PlannerInputStatus3D input_status{PlannerInputStatus3D::kInvalidInput};
  SearchProgress3D progress{SearchProgress3D::kInvalidated};
  std::uint64_t mission_epoch{0U};
  std::uint64_t planned_on_revision{0U};
  std::uint64_t occupied_fingerprint{0U};
  std::uint64_t search_generation{0U};
  std::uint64_t repair_generation{0U};
  std::size_t expansions{0U};
  std::size_t changed_occupied_voxels{0U};
  std::size_t affected_lattice_states{0U};
  std::size_t repair_lattice_states_processed{0U};
  std::size_t repair_lattice_states_pending{0U};
  std::size_t feasibility_expansions{0U};
  std::size_t records{0U};
  std::size_t open_entries{0U};
  std::size_t shortcut_checks{0U};
  std::size_t shortcuts_applied{0U};
  std::size_t lattice_edge_queries{0U};
  std::size_t raw_edge_validation_checks{0U};
  std::size_t adaptive_edge_queries{0U};
  std::size_t adaptive_edges_in_extracted_path{0U};
  std::size_t maximum_queried_lattice_level{0U};
  std::size_t execution_time_search_expansions{0U};
  std::size_t execution_time_search_records{0U};
  std::size_t execution_time_search_open_entries{0U};
  double path_length_m{0.0};
  double remaining_goal_distance_m{0.0};
  double execution_time_search_objective_s{0.0};
  double estimated_execution_time_s{0.0};
  double estimated_translation_time_s{0.0};
  double estimated_stationary_turn_time_s{0.0};
  double world_update_ms{0.0};
  double search_ms{0.0};
  bool invoked{false};
  bool executable{false};
  bool search_state_reused{false};
  bool occupied_world_unchanged{false};
  bool incumbent_retained{false};
  bool repair_pending{false};
  bool feasibility_attempted{false};
  bool feasibility_route_found{false};
  bool execution_time_search_complete{false};
  bool incumbent_available{false};
};

struct ProductionWorldBuildTelemetry3D {
  double build_ms{0.0};
  double esdf_x_pass_ms{0.0};
  double esdf_y_pass_ms{0.0};
  double esdf_z_pass_ms{0.0};
  double esdf_finalize_ms{0.0};
  double conversion_ms{0.0};
  double upload_ms{0.0};
};

struct ProductionRouteMaterializationTelemetry3D {
  double continuation_validation_ms{0.0};
  double route_smoothing_ms{0.0};
  double route_shortcut_validation_ms{0.0};
  double route_corner_validation_ms{0.0};
  double passage_volume_build_ms{0.0};
  double candidate_validation_ms{0.0};
  std::size_t route_shortcuts_applied{0U};
  std::size_t route_corners_smoothed{0U};
  std::size_t route_shortcut_candidates{0U};
  std::size_t route_parallel_shortcut_candidates{0U};
  std::size_t route_corner_candidates{0U};
  std::size_t route_parallel_corner_candidates{0U};
  bool passage_volume_resource_reused{false};
};

struct ProductionRouteSearchProvenance3D {
  ProductionPlanningSearchKind kind{ProductionPlanningSearchKind::kNone};
  RouteInstanceId3D base_route_instance_id{};
  std::optional<double> base_stitch_station_m;
  // A valid ID requires certified overlap with this exact route at activation;
  // an empty ID permits a dynamically validated atomic route handoff.
  RouteInstanceId3D required_splice_base_route_instance_id{};
  Point3 start{};
  Point3 goal{};
  Point3 candidate_endpoint{};
  Vec3 direction{};
  std::size_t candidate_points{0U};
  std::size_t candidate_samples{0U};
};

// Immutable output of geometric route materialization. It deliberately owns no
// planner, world-build, admission, publication, or diagnostics status.
struct MaterializedRoute3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  StaticRouteObjective objective{};
  RouteIntent3D intent{};
  SegmentEvidence3D segment_evidence{};
  std::shared_ptr<const std::vector<RouteSample3D>> route;
  std::shared_ptr<const std::vector<ConstrainedRouteSpan>> constrained_spans;
  std::shared_ptr<const std::vector<PassageVolume>> passage_volumes;
  std::shared_ptr<const std::vector<CooperativePassageAssignment>>
      cooperative_passage_assignments;
  std::shared_ptr<const std::vector<PassageTraversalId>> selected_passage_traversal_ids;
  ProductionRouteSearchProvenance3D provenance{};
  RouteProgressProjection3D initial_projection{};
  std::uint64_t candidate_generation{0U};
  std::uint64_t fingerprint{0U};
  bool reaches_mission_goal{false};
  bool planner_executable{false};
};

// Telemetry is a separate event payload. It is never written back into a world
// or route artifact.
struct ProductionRoutePipelineTelemetry3D {
  ProductionWorldBuildTelemetry3D world_build{};
  ProductionPersistentPlannerTelemetry3D planner{};
  ProductionRouteMaterializationTelemetry3D materialization{};
  double route_search_ms{0.0};
};

struct ProductionMaterializedRouteProposal3D {
  MaterializedRouteProposal3D identity{};
  std::shared_ptr<const CompiledTrajectory3D> trajectory;
  std::shared_ptr<const RouteDecorations3D> decorations;
};

// Admission is a report about a candidate. It does not mutate or own the
// resident world and cannot become an execution owner by itself.
struct RouteAdmissionReport3D {
  StaticRouteCandidateValidation candidate_validation{};
  CertifiedRouteReserveAssessment3D certified_reserve{};
  RouteActivationAssessment3D assessment{};
  RouteProposalReplacementAssessment3D replacement{};
  RouteSuccessorImprovementAssessment3D successor_improvement{};
  DynamicHandoffResult3D handoff{};
  RouteSpliceCertificationResult3D splice{};
  CompiledTrajectoryValidation3D trajectory_validation{};
  RouteDecorationValidation3D decoration_validation{};
  std::optional<PendingRoutePublicationStatus3D> pending_publication_status;
  StaticRouteActivationStatus activation_status{
      StaticRouteActivationStatus::kNotAttempted};
  std::uint64_t snapshot_pose_revision{0U};
  std::uint64_t snapshot_raw_revision{0U};
  std::uint64_t required_objective_sample{0U};
  std::uint64_t tracking_profile_source_occupied_fingerprint{0U};
  std::uint64_t tracking_profile_activation_occupied_fingerprint{0U};
  bool world_compatible{false};
  bool generation_assessed{false};
  bool generation_matches{false};
  bool objective_matches{false};
  bool snapshot_current{false};
  bool resident_world_snapshot_current{false};
  bool objective_snapshot_current{false};
  bool raw_snapshot_current{false};
  bool execution_base_snapshot_current{false};
  bool candidate_world_coherent{false};
  bool certification_execution_base_current{false};
  bool route_certified{false};
  bool trajectory_compile_attempted{false};
  bool trajectory_compiled{false};
  bool observed_world_rebased{false};
  bool publication_world_advanced{false};
  bool certified_pending{false};
  bool commit_assessment_performed{false};
  bool successor_improvement_required{false};
  bool successor_compared_to_pending{false};
  bool pending_snapshot_current{false};

  [[nodiscard]] bool compiledTrajectoryValid() const noexcept;
  [[nodiscard]] bool readyForArbitration(
      const ProductionMaterializedRouteProposal3D& proposal) const noexcept;
};

struct ProductionRouteActivationResult3D {
  MaterializedRoute3D materialized{};
  std::shared_ptr<const CompiledTrajectory3D> trajectory;
  std::shared_ptr<const RouteDecorations3D> decorations;
  std::size_t stop_turn_count{0U};
  ProductionRoutePipelineTelemetry3D telemetry{};
  ProductionMaterializedRouteProposal3D proposal{};
  RouteAdmissionReport3D admission{};
};

} // namespace drone_city_nav
