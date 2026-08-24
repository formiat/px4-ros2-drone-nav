#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {

std::optional<std::chrono::steady_clock::time_point>
ProductionMppiNode::processObservedEsdf3D(const ProductionMppiRawWorld3D& raw_world) {
  const std::shared_ptr<const ObservedOccupancyGrid3D> occupancy = raw_world.occupancy;
  if (!occupancy) {
    RCLCPP_WARN(get_logger(),
                "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                " reason=unavailable_observed_grid",
                raw_world.version.revision);
    return std::nullopt;
  }
  const std::shared_ptr<const VersionedObservedRawWorld3D> execution_owner =
      raw_world.execution_owner;
  if (!execution_owner || !execution_owner->valid() ||
      std::addressof(execution_owner->occupancy()) != occupancy.get() ||
      execution_owner->version().producer_instance_id !=
          raw_world.version.producer_instance_id ||
      execution_owner->version().base_snapshot_revision !=
          raw_world.version.base_snapshot_revision ||
      execution_owner->version().revision != raw_world.version.revision ||
      execution_owner->proprioceptiveFreeSpaceSeed().has_value() ||
      execution_owner->launchSupportContact().has_value()) {
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                 " reason=raw_execution_owner_mismatch",
                 raw_world.version.revision);
    return std::nullopt;
  }

  ProductionMppiNavigation navigation;
  ProductionMppiAppliedControl applied_control;
  ProductionMppiExecutionHorizonOwner execution_horizon_owner;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    applied_control = applied_control_;
    execution_horizon_owner = execution_horizon_owner_;
  }
  if (!navigation.valid) {
    return std::nullopt;
  }

  std::optional<ProductionMppiPreparedEsdf> active_prepared;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    active_prepared = prepared_esdf_;
  }
  const GridBounds3D& world_bounds = occupancy->bounds();
  const Point3 position{navigation.state.x, navigation.state.y, navigation.state.z};
  const std::optional<FootprintBodyAxis> current_body_axis =
      authoritativeBodyAxisForExecution(applied_control, execution_horizon_owner,
                                        navigation, get_clock()->now().nanoseconds(),
                                        maximum_control_feedback_age_ms_,
                                        maximum_pose_age_ms_);
  const std::optional<ProprioceptiveFreeSpaceSeed3D> free_space_seed =
      current_body_axis.has_value()
          ? std::optional<ProprioceptiveFreeSpaceSeed3D>{ProprioceptiveFreeSpaceSeed3D{
                .position = position,
                .body_axis = *current_body_axis,
                .footprint = physical_footprint_config_,
            }}
          : std::nullopt;
  if (!launch_support_seed_ && free_space_seed.has_value()) {
    launch_support_seed_ = free_space_seed;
  }
  if (!launch_support_evaluated_ && launch_support_seed_.has_value()) {
    const bool vehicle_land_contact_received =
        vehicle_land_contact_received_.load(std::memory_order_acquire);
    const bool vehicle_launch_support_confirmed =
        launch_support_confirmed_by_land_detector_.load(std::memory_order_acquire);
    if (vehicle_launch_support_confirmed) {
      launch_support_contact_ =
          makeVehicleLandedSupportContact3D(occupancy->bounds(), *launch_support_seed_);
    } else {
      launch_support_contact_ =
          detectLaunchSupportContact3D(*occupancy, *launch_support_seed_);
      if (!launch_support_contact_ && vehicle_land_contact_received) {
        const SweptFootprintResult without_support = validateRawFootprintAt(
            *occupancy, launch_support_seed_->position, launch_support_seed_->body_axis,
            physical_footprint_config_);
        if (without_support.accepted()) {
          launch_support_evaluated_ = true;
          RCLCPP_INFO(get_logger(),
                      "LAUNCH_SUPPORT_CONTACT state=not_present source="
                      "observed_known_free revision=%" PRIu64,
                      raw_world.version.revision);
        }
      }
    }
    if (launch_support_contact_) {
      launch_support_evaluated_ = true;
      RCLCPP_INFO(get_logger(),
                  "LAUNCH_SUPPORT_CONTACT state=active revision=%" PRIu64
                  " source=%s cells=%zu occupied_evidence=%zu"
                  " anchor=(%.3f,%.3f,%.3f)",
                  raw_world.version.revision,
                  launch_support_contact_->evidence_source ==
                          LaunchSupportEvidenceSource::kVehicleLandDetector
                      ? "vehicle_land_detector"
                      : "observed_occupancy",
                  launch_support_contact_->contact_cells.size(),
                  launch_support_contact_->occupied_evidence_cells,
                  launch_support_seed_->position.x, launch_support_seed_->position.y,
                  launch_support_seed_->position.z);
    } else if (!launch_support_evaluated_ &&
               distance3D(position, launch_support_seed_->position) >
                   std::max(0.5, physical_footprint_config_.radius_m)) {
      launch_support_evaluated_ = true;
      RCLCPP_INFO(get_logger(),
                  "LAUNCH_SUPPORT_CONTACT state=not_detected_after_departure"
                  " revision=%" PRIu64 " position=(%.3f,%.3f,%.3f)",
                  raw_world.version.revision, position.x, position.y, position.z);
    } else if (!launch_support_evaluated_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LAUNCH_SUPPORT_CONTACT state=awaiting_evidence revision=%" PRIu64
          " anchor=(%.3f,%.3f,%.3f)",
          raw_world.version.revision, launch_support_seed_->position.x,
          launch_support_seed_->position.y, launch_support_seed_->position.z);
    }
  }
  if (launch_support_contact_) {
    if (updateLaunchSupportSettling(*launch_support_contact_, position)) {
      RCLCPP_INFO(get_logger(),
                  "LAUNCH_SUPPORT_CONTACT state=settled revision=%" PRIu64
                  " minimum_axial_departure_m=%.3f position=(%.3f,%.3f,%.3f)",
                  raw_world.version.revision,
                  launch_support_contact_->minimum_axial_departure_m, position.x,
                  position.y, position.z);
    }
    if (current_body_axis.has_value() && free_space_seed.has_value()) {
      const SweptFootprintResult without_support =
          validateRawFootprintAt(*occupancy, position, *current_body_axis,
                                 physical_footprint_config_, &*free_space_seed);
      const FootprintBodyAxis support_axis = launch_support_contact_->seed.body_axis;
      const Point3 support_delta{
          position.x - launch_support_contact_->seed.position.x,
          position.y - launch_support_contact_->seed.position.y,
          position.z - launch_support_contact_->seed.position.z,
      };
      const double support_axial_departure_m = support_delta.x * support_axis.x +
                                               support_delta.y * support_axis.y +
                                               support_delta.z * support_axis.z;
      if (without_support.accepted() &&
          support_axial_departure_m > occupancy->bounds().resolution_m) {
        RCLCPP_INFO(get_logger(),
                    "LAUNCH_SUPPORT_CONTACT state=released revision=%" PRIu64
                    " axial_departure_m=%.3f position=(%.3f,%.3f,%.3f)",
                    raw_world.version.revision, support_axial_departure_m, position.x,
                    position.y, position.z);
        launch_support_contact_.reset();
      }
    }
  }
  const LaunchSupportContact3D* const launch_support_contact =
      launch_support_contact_ ? &*launch_support_contact_ : nullptr;
  const std::optional<SweptFootprintResult> current_footprint =
      current_body_axis.has_value() && free_space_seed.has_value()
          ? std::optional<SweptFootprintResult>{validateRawFootprintAt(
                *occupancy, position, *current_body_axis, physical_footprint_config_,
                &*free_space_seed, launch_support_contact)}
          : std::nullopt;
  double support_axial_departure_m{0.0};
  double support_lateral_departure_m{0.0};
  bool failure_is_launch_support_cell{false};
  if (launch_support_contact != nullptr && current_footprint.has_value()) {
    const FootprintBodyAxis axis = launch_support_contact->seed.body_axis;
    const Point3 delta{position.x - launch_support_contact->seed.position.x,
                       position.y - launch_support_contact->seed.position.y,
                       position.z - launch_support_contact->seed.position.z};
    support_axial_departure_m = delta.x * axis.x + delta.y * axis.y + delta.z * axis.z;
    support_lateral_departure_m = std::sqrt(
        std::max(0.0, delta.x * delta.x + delta.y * delta.y + delta.z * delta.z -
                          support_axial_departure_m * support_axial_departure_m));
    constexpr double kCellContainmentToleranceM{1.0e-6};
    failure_is_launch_support_cell = std::ranges::any_of(
        launch_support_contact->contact_cells, [&](const AxisAlignedBox3D& cell) {
          return current_footprint->failure_point.x >=
                     cell.minimum.x - kCellContainmentToleranceM &&
                 current_footprint->failure_point.x <=
                     cell.maximum.x + kCellContainmentToleranceM &&
                 current_footprint->failure_point.y >=
                     cell.minimum.y - kCellContainmentToleranceM &&
                 current_footprint->failure_point.y <=
                     cell.maximum.y + kCellContainmentToleranceM &&
                 current_footprint->failure_point.z >=
                     cell.minimum.z - kCellContainmentToleranceM &&
                 current_footprint->failure_point.z <=
                     cell.maximum.z + kCellContainmentToleranceM;
        });
  }
  if (current_footprint.has_value()) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "OBSERVED_FOOTPRINT_READINESS revision=%" PRIu64
        " status=%s position=(%.3f,%.3f,%.3f) failure_point=(%.3f,%.3f,%.3f)"
        " launch_support_active=%s support_failure_cell=%s"
        " support_axial_departure_m=%.3f support_lateral_departure_m=%.3f"
        " support_maximum_lateral_departure_m=%.3f"
        " support_minimum_axial_departure_m=%.3f"
        " support_maximum_axial_settling_m=%.3f",
        raw_world.version.revision, sweptFootprintStatusName(current_footprint->status),
        position.x, position.y, position.z, current_footprint->failure_point.x,
        current_footprint->failure_point.y, current_footprint->failure_point.z,
        launch_support_contact != nullptr ? "true" : "false",
        failure_is_launch_support_cell ? "true" : "false", support_axial_departure_m,
        support_lateral_departure_m,
        launch_support_contact != nullptr
            ? launch_support_contact->maximum_lateral_departure_m
            : 0.0,
        launch_support_contact != nullptr
            ? launch_support_contact->minimum_axial_departure_m
            : 0.0,
        launch_support_contact != nullptr
            ? launch_support_contact->maximum_axial_settling_m
            : 0.0);
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "OBSERVED_FOOTPRINT_READINESS revision=%" PRIu64
                         " status=body_axis_unavailable position=(%.3f,%.3f,%.3f)"
                         " proprioceptive_free_space=false",
                         raw_world.version.revision, position.x, position.y,
                         position.z);
  }
  GridBounds3D local_bounds;
  bool recenter = true;
  if (active_prepared && active_prepared->distances_m &&
      active_prepared->grid.depth > 1 && active_prepared->grid.outside_is_unknown) {
    local_bounds = GridBounds3D{
        .origin_x = active_prepared->grid.origin_x_m,
        .origin_y = active_prepared->grid.origin_y_m,
        .origin_z = active_prepared->grid.origin_z_m,
        .resolution_m = active_prepared->grid.resolution_m,
        .width_cells = active_prepared->grid.width,
        .height_cells = active_prepared->grid.height,
        .depth_cells = active_prepared->grid.depth,
    };
    recenter = localObservedEsdfNeedsRecenter(local_bounds, world_bounds, position,
                                              no_static_3d_esdf_window_);
  }
  if (recenter) {
    local_bounds = selectLocalObservedEsdfBounds(world_bounds, position,
                                                 no_static_3d_esdf_window_);
  }

  const std::uint64_t local_fingerprint =
      observedOccupancyFingerprint(*occupancy, local_bounds);
  const bool launch_support_resolution_pending = !launch_support_evaluated_;
  const bool free_space_seed_unchanged =
      active_prepared &&
      active_prepared->proprioceptive_free_space_seed.has_value() ==
          free_space_seed.has_value() &&
      (!free_space_seed.has_value() ||
       sameProprioceptiveFreeSpaceSeed3D(
           *active_prepared->proprioceptive_free_space_seed, *free_space_seed));
  const bool launch_support_unchanged =
      active_prepared &&
      active_prepared->launch_support_contact.has_value() ==
          launch_support_contact_.has_value() &&
      (!launch_support_contact_.has_value() ||
       sameLaunchSupportContact3D(*active_prepared->launch_support_contact,
                                  *launch_support_contact_)) &&
      active_prepared->launch_support_resolution_pending ==
          launch_support_resolution_pending;
  const bool execution_evidence_unchanged =
      free_space_seed_unchanged && launch_support_unchanged;
  if (active_prepared && !execution_evidence_unchanged) {
    {
      const std::scoped_lock lock{world_generation_publication_mutex_,
                                  esdf_state_mutex_};
      prepared_esdf_.reset();
    }
    {
      const std::scoped_lock lock{guide_queue_mutex_};
      pending_guide_world_.reset();
    }
    active_prepared.reset();
    RCLCPP_INFO(get_logger(),
                "EXECUTION_EVIDENCE_WORLD_INVALIDATED raw_revision=%" PRIu64
                " free_space_seed=%s support_active=%s resolution_pending=%s",
                raw_world.version.revision,
                free_space_seed.has_value() ? "true" : "false",
                launch_support_contact != nullptr ? "true" : "false",
                launch_support_resolution_pending ? "true" : "false");
  }
  const bool local_occupancy_unchanged =
      active_prepared && active_prepared->distances_m && !recenter &&
      active_prepared->source_occupied_fingerprint == local_fingerprint &&
      execution_evidence_unchanged;
  const auto build_started_at = std::chrono::steady_clock::now();
  const bool first_build =
      no_static_esdf_last_build_time_ == std::chrono::steady_clock::time_point{};
  const bool build_rate_due =
      first_build ||
      std::chrono::duration<double>(build_started_at - no_static_esdf_last_build_time_)
              .count() >= 1.0 / no_static_3d_esdf_update_rate_hz_;
  if (local_occupancy_unchanged) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NO_STATIC_ESDF3D_DEFERRED raw_revision=%" PRIu64
        " reason=observed_unchanged reconstruction_ms=%.2f raw_updates=%" PRIu64
        " builds=%" PRIu64 " throttled=%" PRIu64,
        raw_world.version.revision, raw_world.reconstruction_ms,
        no_static_raw_updates_.load(std::memory_order_relaxed),
        no_static_esdf_builds_.load(std::memory_order_relaxed),
        no_static_esdf_throttled_updates_.load(std::memory_order_relaxed));
    return std::nullopt;
  }
  if (active_prepared && !recenter && !build_rate_due) {
    no_static_esdf_throttled_updates_.fetch_add(1U, std::memory_order_relaxed);
    const auto update_period =
        std::chrono::duration<double>{1.0 / no_static_3d_esdf_update_rate_hz_};
    const auto retry_not_before =
        no_static_esdf_last_build_time_ +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(update_period);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NO_STATIC_ESDF3D_DEFERRED raw_revision=%" PRIu64
        " reason=rate_limited reconstruction_ms=%.2f raw_updates=%" PRIu64
        " builds=%" PRIu64 " throttled=%" PRIu64,
        raw_world.version.revision, raw_world.reconstruction_ms,
        no_static_raw_updates_.load(std::memory_order_relaxed),
        no_static_esdf_builds_.load(std::memory_order_relaxed),
        no_static_esdf_throttled_updates_.load(std::memory_order_relaxed));
    return retry_not_before;
  }

  const std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world_owner =
      execution_owner->deriveRouteEvidence(free_space_seed, launch_support_contact_);
  if (!observed_raw_world_owner ||
      !execution_owner->sharesObservationOwner(*observed_raw_world_owner) ||
      std::addressof(observed_raw_world_owner->occupancy()) != occupancy.get() ||
      observed_raw_world_owner->occupiedSnapshot() !=
          execution_owner->occupiedSnapshot()) {
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                 " reason=route_evidence_derivation_failed",
                 raw_world.version.revision);
    return std::nullopt;
  }

  ObservedEsdf3D field = buildObservedEsdf3D(
      *occupancy, local_bounds,
      static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0,
      planning_worker_pool_.get(),
      free_space_seed.has_value() ? std::addressof(*free_space_seed) : nullptr,
      launch_support_contact);
  auto host_distances =
      std::make_shared<const std::vector<float>>(std::move(field.distances_m));

  ProductionMppiPreparedEsdf world_update;
  world_update.producer_instance_id = raw_world.version.producer_instance_id;
  world_update.revision = field.occupancy_fingerprint;
  world_update.source_raw_revision = raw_world.version.revision;
  world_update.source_occupied_fingerprint = local_fingerprint;
  world_update.source_stamp_ns = raw_world.ready_stamp_ns;
  world_update.ready_stamp_ns = get_clock()->now().nanoseconds();
  world_update.build_ms =
      field.stats.distance_field.duration_ms + field.stats.classification_ms;
  world_update.esdf_x_pass_ms = field.stats.distance_field.x_pass_ms;
  world_update.esdf_y_pass_ms = field.stats.distance_field.y_pass_ms;
  world_update.esdf_z_pass_ms = field.stats.distance_field.z_pass_ms;
  world_update.esdf_finalize_ms = field.stats.distance_field.finalize_ms;
  world_update.conversion_ms =
      raw_world.reconstruction_ms + field.stats.classification_ms;
  world_update.grid = field.grid;
  world_update.distances_m = host_distances;
  world_update.observed_occupancy = occupancy;
  world_update.observed_raw_world_owner = observed_raw_world_owner;
  world_update.proprioceptive_free_space_seed = free_space_seed;
  world_update.launch_support_contact = launch_support_contact_;
  world_update.launch_support_resolution_pending = launch_support_resolution_pending;

  std::shared_ptr<const IncrementalTopologyGraph3DSnapshot> latest_topology_graph;
  IncrementalTopologyGraph3DUpdate latest_topology_graph_update;
  {
    const std::scoped_lock lock{topology_state_mutex_};
    if (latest_observed_topological_producer_instance_id_ ==
            raw_world.version.producer_instance_id &&
        latest_observed_topological_graph_ &&
        latest_observed_topological_graph_->revision() <= raw_world.version.revision) {
      latest_topology_graph = latest_observed_topological_graph_;
      latest_topology_graph_update = latest_observed_topological_graph_update_;
    }
  }

  const std::shared_ptr<const ProductionNavigationObjective> current_objective =
      navigationObjective();
  ProductionMppiPreparedEsdf prepared;
  bool local_world_generation_valid{false};
  std::unique_lock generation_lock{world_generation_publication_mutex_};
  const mppi::EsdfUploadResult upload = engine_->updateEsdf(
      mppi::EsdfSnapshot{field.grid, *host_distances, field.occupancy_fingerprint});
  if (!upload.accepted) {
    return std::nullopt;
  }
  world_update.upload_ms = upload.upload_ms;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (prepared_esdf_) {
      prepared = *prepared_esdf_;
    }

    // Route activation and coherent-world publication share this mutex. Merge the
    // completed world build into the latest resident route state so a build that
    // started before activation cannot restore an older route generation.
    const bool resident_topology_is_compatible =
        prepared.topological_graph &&
        prepared.producer_instance_id == raw_world.version.producer_instance_id &&
        prepared.topology_source_raw_revision <= raw_world.version.revision;
    const bool latest_topology_is_newer =
        latest_topology_graph &&
        (!resident_topology_is_compatible ||
         latest_topology_graph->revision() >= prepared.topology_source_raw_revision);
    if (latest_topology_is_newer) {
      world_update.topological_graph = std::move(latest_topology_graph);
      world_update.topological_graph_update = latest_topology_graph_update;
      world_update.topology_source_raw_revision =
          world_update.topological_graph->revision();
    } else if (resident_topology_is_compatible) {
      world_update.topological_graph = prepared.topological_graph;
      world_update.topological_graph_update = prepared.topological_graph_update;
      world_update.topology_source_raw_revision = prepared.topology_source_raw_revision;
    }

    const std::uint64_t topology_revision =
        world_update.topological_graph ? world_update.topological_graph->revision()
                                       : 0U;
    const std::optional<LocalWorldGeneration> local_world_generation =
        local_world_generation_counter_.issue(raw_world.version, navigation.revision,
                                              world_update.revision, upload.revision,
                                              topology_revision);
    if (local_world_generation.has_value()) {
      world_update.local_world_generation = *local_world_generation;
      adoptWorldResources(prepared, world_update);
      if (current_objective) {
        prepared.search_objective = makeStaticRouteObjective(*current_objective);
      }
      prepared.lattice_search_performed = false;
      prepared.lattice_continuation_attempt = 0U;
      local_world_generation_valid = productionWorldGenerationCoherent(prepared);
      if (local_world_generation_valid) {
        prepared_esdf_ = prepared;
      }
    }
    if (!local_world_generation_valid) {
      prepared_esdf_.reset();
    }
  }
  generation_lock.unlock();
  if (!local_world_generation_valid) {
    rejected_world_generation_publications_.fetch_add(1U, std::memory_order_relaxed);
    if (world_ready_.exchange(false, std::memory_order_acq_rel)) {
      publishWorldReadiness(false);
    }
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_ESDF3D_ONLINE rejected raw_revision=%" PRIu64
                 " reason=mixed_local_world_generation",
                 raw_world.version.revision);
    return std::nullopt;
  }
  no_static_esdf_last_build_time_ = build_started_at;
  no_static_esdf_builds_.fetch_add(1U, std::memory_order_relaxed);
  const std::uint64_t blocked_raw_revision =
      observed_route_blocked_raw_revision_.load(std::memory_order_acquire);
  const std::uint64_t dispatched_raw_revision =
      observed_route_replan_dispatched_raw_revision_.load(std::memory_order_acquire);
  if (blocked_raw_revision != 0U &&
      blocked_raw_revision <= raw_world.version.revision &&
      dispatched_raw_revision < blocked_raw_revision &&
      prepared.global_guide_generation != 0U) {
    observed_route_replan_dispatched_raw_revision_.store(blocked_raw_revision,
                                                         std::memory_order_release);
    RCLCPP_INFO(get_logger(),
                "OBSERVED_ROUTE_REPLAN status=esdf_caught_up raw_revision=%" PRIu64
                " esdf_revision=%" PRIu64 " generation=%" PRIu64,
                raw_world.version.revision, prepared.revision,
                prepared.global_guide_generation);
    requestStaticRouteReplan(GlobalGuideReleaseReason::kBlocked,
                             prepared.global_guide_generation);
  }
  const bool initial_route_search_required = prepared.global_guide_generation == 0U;
  bool initial_route_search_queued = false;
  bool initial_route_search_already_pending = false;
  if (initial_route_search_required) {
    auto guide_world = std::make_shared<const ProductionMppiPreparedEsdf>(prepared);
    {
      const std::scoped_lock lock{guide_queue_mutex_};
      if (pending_guide_world_) {
        initial_route_search_already_pending = true;
      } else {
        pending_guide_world_ = std::move(guide_world);
        initial_route_search_queued = true;
      }
    }
    if (initial_route_search_queued) {
      guide_queue_condition_.notify_all();
    }
  }
  if (!world_ready_.exchange(true, std::memory_order_acq_rel)) {
    publishWorldReadiness(true);
  }

  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_ESDF3D_ONLINE revision=%" PRIu64 " raw_revision=%" PRIu64
      " build_ms=%.2f classify_ms=%.2f upload_ms=%.2f dimensions=%dx%dx%d "
      "known=%zu free=%zu occupied=%zu unknown=%zu proprioceptive_free=%zu "
      "launch_support=%zu "
      "recenter=%s local_world_generation=%" PRIu64 " route_generation=%" PRIu64
      " route_search=%s builds=%" PRIu64 " throttled=%" PRIu64 " dropped_raw=%" PRIu64,
      prepared.revision, raw_world.version.revision, prepared.build_ms,
      field.stats.classification_ms, prepared.upload_ms, prepared.grid.width,
      prepared.grid.height, prepared.grid.depth, field.stats.known_voxels,
      field.stats.free_voxels, field.stats.occupied_voxels, field.stats.unknown_voxels,
      field.stats.proprioceptive_free_voxels, field.stats.launch_support_voxels,
      recenter ? "true" : "false", prepared.local_world_generation.generation,
      prepared.global_guide_generation,
      !initial_route_search_required
          ? "active_route_preserved"
          : (initial_route_search_queued
                 ? "initial_queued"
                 : (initial_route_search_already_pending ? "initial_already_pending"
                                                         : "initial_not_queued")),
      no_static_esdf_builds_.load(std::memory_order_relaxed),
      no_static_esdf_throttled_updates_.load(std::memory_order_relaxed),
      dropped_raw_snapshots_.load(std::memory_order_relaxed));
  return std::nullopt;
}

} // namespace drone_city_nav
