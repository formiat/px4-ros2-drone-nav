#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "production_mppi_node.hpp"

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

  ProductionMppiNavigation navigation;
  ProductionMppiAppliedControl applied_control;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    applied_control = applied_control_;
  }
  const std::shared_ptr<const ProductionNavigationObjective> build_objective =
      navigationObjective();
  std::shared_ptr<const IncrementalTopologyGraph3DSnapshot> topology_graph;
  IncrementalTopologyGraph3DUpdate topology_graph_update;
  {
    const std::scoped_lock lock{topology_state_mutex_};
    if (latest_observed_topological_producer_instance_id_ ==
            raw_world.version.producer_instance_id &&
        latest_observed_topological_graph_ &&
        latest_observed_topological_graph_->revision() <= raw_world.version.revision) {
      topology_graph = latest_observed_topological_graph_;
      topology_graph_update = latest_observed_topological_graph_update_;
    }
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
  const FootprintBodyAxis current_body_axis =
      applied_control.valid
          ? bodyAxisFromWorldAcceleration(Vec3{applied_control.control.ax,
                                               applied_control.control.ay,
                                               applied_control.control.az})
          : FootprintBodyAxis{};
  const ProprioceptiveFreeSpaceSeed3D free_space_seed{
      .position = position,
      .body_axis = current_body_axis,
      .footprint = physical_footprint_config_,
  };
  if (!launch_support_seed_) {
    launch_support_seed_ = free_space_seed;
  }
  if (!launch_support_evaluated_) {
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
    const SweptFootprintResult without_support =
        validateRawFootprintAt(*occupancy, position, current_body_axis,
                               physical_footprint_config_, &free_space_seed);
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
  const LaunchSupportContact3D* const launch_support_contact =
      launch_support_contact_ ? &*launch_support_contact_ : nullptr;
  const SweptFootprintResult current_footprint = validateRawFootprintAt(
      *occupancy, position, current_body_axis, physical_footprint_config_,
      &free_space_seed, launch_support_contact);
  double support_axial_departure_m{0.0};
  double support_lateral_departure_m{0.0};
  bool failure_is_launch_support_cell{false};
  if (launch_support_contact != nullptr) {
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
          return current_footprint.failure_point.x >=
                     cell.minimum.x - kCellContainmentToleranceM &&
                 current_footprint.failure_point.x <=
                     cell.maximum.x + kCellContainmentToleranceM &&
                 current_footprint.failure_point.y >=
                     cell.minimum.y - kCellContainmentToleranceM &&
                 current_footprint.failure_point.y <=
                     cell.maximum.y + kCellContainmentToleranceM &&
                 current_footprint.failure_point.z >=
                     cell.minimum.z - kCellContainmentToleranceM &&
                 current_footprint.failure_point.z <=
                     cell.maximum.z + kCellContainmentToleranceM;
        });
  }
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "OBSERVED_FOOTPRINT_READINESS revision=%" PRIu64
      " status=%s position=(%.3f,%.3f,%.3f) failure_point=(%.3f,%.3f,%.3f)"
      " launch_support_active=%s support_failure_cell=%s"
      " support_axial_departure_m=%.3f support_lateral_departure_m=%.3f"
      " support_maximum_lateral_departure_m=%.3f"
      " support_minimum_axial_departure_m=%.3f"
      " support_maximum_axial_settling_m=%.3f",
      raw_world.version.revision, sweptFootprintStatusName(current_footprint.status),
      position.x, position.y, position.z, current_footprint.failure_point.x,
      current_footprint.failure_point.y, current_footprint.failure_point.z,
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
  const bool launch_support_unchanged =
      active_prepared &&
      active_prepared->launch_support_contact.has_value() ==
          (launch_support_contact != nullptr) &&
      active_prepared->launch_support_resolution_pending ==
          launch_support_resolution_pending;
  if (active_prepared && !launch_support_unchanged) {
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      prepared_esdf_.reset();
    }
    {
      const std::scoped_lock lock{guide_queue_mutex_};
      pending_guide_world_.reset();
    }
    active_prepared.reset();
    RCLCPP_INFO(get_logger(),
                "LAUNCH_SUPPORT_WORLD_INVALIDATED raw_revision=%" PRIu64
                " support_active=%s resolution_pending=%s",
                raw_world.version.revision,
                launch_support_contact != nullptr ? "true" : "false",
                launch_support_resolution_pending ? "true" : "false");
  }
  const bool local_occupancy_unchanged =
      active_prepared && active_prepared->distances_m && !recenter &&
      active_prepared->source_occupied_fingerprint == local_fingerprint &&
      launch_support_unchanged;
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

  ObservedEsdf3D field = buildObservedEsdf3D(
      *occupancy, local_bounds,
      static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0,
      planning_worker_pool_.get(), &free_space_seed, launch_support_contact);
  const mppi::EsdfUploadResult upload = engine_->updateEsdf(
      mppi::EsdfSnapshot{field.grid, field.distances_m, field.occupancy_fingerprint});
  if (!upload.accepted) {
    return std::nullopt;
  }
  auto host_distances =
      std::make_shared<const std::vector<float>>(std::move(field.distances_m));
  no_static_esdf_last_build_time_ = build_started_at;
  no_static_esdf_builds_.fetch_add(1U, std::memory_order_relaxed);

  ProductionMppiPreparedEsdf prepared;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (prepared_esdf_) {
      prepared = *prepared_esdf_;
    }
  }
  const bool retained_topology_is_compatible =
      !prepared.topological_graph ||
      (prepared.producer_instance_id == raw_world.version.producer_instance_id &&
       prepared.topology_source_raw_revision <= raw_world.version.revision);
  prepared.producer_instance_id = raw_world.version.producer_instance_id;
  prepared.revision = field.occupancy_fingerprint;
  prepared.source_raw_revision = raw_world.version.revision;
  prepared.source_occupied_fingerprint = local_fingerprint;
  prepared.source_stamp_ns = raw_world.ready_stamp_ns;
  prepared.ready_stamp_ns = get_clock()->now().nanoseconds();
  prepared.build_ms =
      field.stats.distance_field.duration_ms + field.stats.classification_ms;
  prepared.esdf_x_pass_ms = field.stats.distance_field.x_pass_ms;
  prepared.esdf_y_pass_ms = field.stats.distance_field.y_pass_ms;
  prepared.esdf_z_pass_ms = field.stats.distance_field.z_pass_ms;
  prepared.esdf_finalize_ms = field.stats.distance_field.finalize_ms;
  prepared.conversion_ms = raw_world.reconstruction_ms + field.stats.classification_ms;
  prepared.upload_ms = upload.upload_ms;
  prepared.grid = field.grid;
  prepared.distances_m = host_distances;
  prepared.raw_occupancy.reset();
  prepared.observed_occupancy = occupancy;
  prepared.proprioceptive_free_space_seed = free_space_seed;
  prepared.launch_support_contact = launch_support_contact_;
  prepared.launch_support_resolution_pending = launch_support_resolution_pending;
  // Topology rebuilds asynchronously from the same revisioned observation stream.
  // Keep the latest compatible graph while the worker catches up with this ESDF
  // snapshot; replacing it with nullptr would make the fallback unavailable on
  // every intervening local ESDF refresh.
  if (topology_graph) {
    prepared.topological_graph = std::move(topology_graph);
    prepared.topological_graph_update = topology_graph_update;
    prepared.topology_source_raw_revision = prepared.topological_graph->revision();
  } else if (!retained_topology_is_compatible) {
    prepared.topological_graph.reset();
    prepared.topological_graph_update = {};
    prepared.topology_source_raw_revision = 0U;
  }
  if (const std::shared_ptr<const ProductionNavigationObjective> objective =
          navigationObjective()) {
    prepared.search_objective = makeStaticRouteObjective(*objective);
  }
  prepared.lattice_search_performed = false;
  prepared.lattice_continuation_attempt = 0U;
  const std::uint64_t topology_revision =
      prepared.topological_graph ? prepared.topological_graph->revision() : 0U;
  const std::optional<LocalWorldGeneration> local_world_generation =
      local_world_generation_counter_.issue(raw_world.version, navigation.revision,
                                            prepared.revision, upload.revision,
                                            topology_revision);
  if (!local_world_generation.has_value()) {
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_ESDF3D_ONLINE rejected raw_revision=%" PRIu64
                 " reason=invalid_local_world_generation",
                 raw_world.version.revision);
    return std::nullopt;
  }
  prepared.local_world_generation = *local_world_generation;

  {
    const std::scoped_lock lock{esdf_state_mutex_};
    prepared_esdf_ = prepared;
  }
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

  RCLCPP_INFO(get_logger(),
              "PRODUCTION_MPPI_ESDF3D_ONLINE revision=%" PRIu64 " raw_revision=%" PRIu64
              " build_ms=%.2f classify_ms=%.2f upload_ms=%.2f dimensions=%dx%dx%d "
              "known=%zu free=%zu occupied=%zu unknown=%zu proprioceptive_free=%zu "
              "launch_support=%zu "
              "recenter=%s route_search=%s builds=%" PRIu64 " throttled=%" PRIu64
              " dropped_raw=%" PRIu64,
              prepared.revision, raw_world.version.revision, prepared.build_ms,
              field.stats.classification_ms, prepared.upload_ms, prepared.grid.width,
              prepared.grid.height, prepared.grid.depth, field.stats.known_voxels,
              field.stats.free_voxels, field.stats.occupied_voxels,
              field.stats.unknown_voxels, field.stats.proprioceptive_free_voxels,
              field.stats.launch_support_voxels, recenter ? "true" : "false",
              !initial_route_search_required
                  ? "active_route_preserved"
                  : (initial_route_search_queued ? "initial_queued"
                                                 : (initial_route_search_already_pending
                                                        ? "initial_already_pending"
                                                        : "initial_not_queued")),
              no_static_esdf_builds_.load(std::memory_order_relaxed),
              no_static_esdf_throttled_updates_.load(std::memory_order_relaxed),
              dropped_raw_snapshots_.load(std::memory_order_relaxed));
  return std::nullopt;
}

} // namespace drone_city_nav
