#include "drone_city_nav/observed_esdf_3d.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <memory>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::processObservedEsdf3D(
    const ProductionMppiRawWorld3D& raw_world) {
  const std::shared_ptr<const ObservedOccupancyGrid3D> occupancy = raw_world.occupancy;
  if (!occupancy) {
    RCLCPP_WARN(get_logger(),
                "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                " reason=unavailable_observed_grid",
                raw_world.revision);
    return;
  }

  const auto topology_started = std::chrono::steady_clock::now();
  const IncrementalTopologicalWorldUpdate3D topology_update =
      topological_navigation_3d_->updateObserved(
          *occupancy, raw_world.producer_instance_id, raw_world.revision,
          raw_world.dirty_chunks, raw_world.full_reset);
  const double topology_update_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                topology_started)
          .count();
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (prepared_esdf_) {
      prepared_esdf_->topological_graph = topology_update.snapshot;
      prepared_esdf_->topological_graph_update = topology_update.graph;
    }
  }
  RCLCPP_INFO(
      get_logger(),
      "INCREMENTAL_TOPOLOGY3D_UPDATE revision=%" PRIu64
      " full_reset=%s dirty_chunks=%zu discovered_dirty_tiles=%zu "
      "rebuilt_tiles=%zu pending_tiles=%zu refined_tiles=%zu "
      "base_resolution_m=%.3f coarse_resolution_m=%.3f "
      "refined_resolution_m=%.3f sampled_cells=%zu retained_nodes=%zu "
      "created_nodes=%zu retired_nodes=%zu nodes=%zu edges=%zu update_ms=%.2f",
      topology_update.graph.revision,
      topology_update.graph.full_reset ? "true" : "false",
      topology_update.graph.requested_dirty_chunks,
      topology_update.graph.discovered_dirty_tiles, topology_update.graph.rebuilt_tiles,
      topology_update.graph.pending_tiles,
      topology_update.graph.adaptively_refined_tiles, occupancy->bounds().resolution_m,
      occupancy->bounds().resolution_m *
          topological_graph_3d_config_.coarse_sample_stride_cells,
      occupancy->bounds().resolution_m *
          topological_graph_3d_config_.refined_sample_stride_cells,
      topology_update.graph.sampled_navigable_cells,
      topology_update.graph.retained_node_ids, topology_update.graph.created_nodes,
      topology_update.graph.retired_nodes, topology_update.graph.node_count,
      topology_update.graph.edge_count, topology_update_ms);

  ProductionMppiNavigation navigation;
  ProductionMppiAppliedControl applied_control;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    applied_control = applied_control_;
  }
  if (!navigation.valid) {
    return;
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
                      raw_world.revision);
        }
      }
    }
    if (launch_support_contact_) {
      launch_support_evaluated_ = true;
      RCLCPP_INFO(get_logger(),
                  "LAUNCH_SUPPORT_CONTACT state=active revision=%" PRIu64
                  " source=%s cells=%zu occupied_evidence=%zu"
                  " anchor=(%.3f,%.3f,%.3f)",
                  raw_world.revision,
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
                  raw_world.revision, position.x, position.y, position.z);
    } else if (!launch_support_evaluated_) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "LAUNCH_SUPPORT_CONTACT state=awaiting_evidence revision=%" PRIu64
          " anchor=(%.3f,%.3f,%.3f)",
          raw_world.revision, launch_support_seed_->position.x,
          launch_support_seed_->position.y, launch_support_seed_->position.z);
    }
  }
  if (launch_support_contact_) {
    if (updateLaunchSupportSettling(*launch_support_contact_, position)) {
      RCLCPP_INFO(get_logger(),
                  "LAUNCH_SUPPORT_CONTACT state=settled revision=%" PRIu64
                  " minimum_axial_departure_m=%.3f position=(%.3f,%.3f,%.3f)",
                  raw_world.revision,
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
                  raw_world.revision, support_axial_departure_m, position.x, position.y,
                  position.z);
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
      raw_world.revision, sweptFootprintStatusName(current_footprint.status),
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
                                              no_static_3d_esdf_recenter_margin_m_);
  }
  if (recenter) {
    local_bounds = selectLocalObservedEsdfBounds(world_bounds, position,
                                                 no_static_3d_esdf_half_extent_m_);
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
  if (local_occupancy_unchanged || (active_prepared && !recenter && !build_rate_due)) {
    if (!local_occupancy_unchanged) {
      no_static_esdf_throttled_updates_.fetch_add(1U, std::memory_order_relaxed);
    } else {
      const std::scoped_lock lock{esdf_state_mutex_};
      if (prepared_esdf_ && prepared_esdf_->revision == active_prepared->revision) {
        prepared_esdf_->source_stamp_ns = raw_world.ready_stamp_ns;
        prepared_esdf_->ready_stamp_ns = raw_world.ready_stamp_ns;
      }
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NO_STATIC_ESDF3D_DEFERRED raw_revision=%" PRIu64
        " reason=%s reconstruction_ms=%.2f raw_updates=%" PRIu64 " builds=%" PRIu64
        " throttled=%" PRIu64,
        raw_world.revision,
        local_occupancy_unchanged ? "observed_unchanged" : "rate_limited",
        raw_world.reconstruction_ms,
        no_static_raw_updates_.load(std::memory_order_relaxed),
        no_static_esdf_builds_.load(std::memory_order_relaxed),
        no_static_esdf_throttled_updates_.load(std::memory_order_relaxed));
    return;
  }

  ObservedEsdf3D field = buildObservedEsdf3D(
      *occupancy, local_bounds,
      static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0,
      planning_worker_pool_.get(), &free_space_seed, launch_support_contact);
  const mppi::EsdfUploadResult upload = engine_->updateEsdf(
      mppi::EsdfSnapshot{field.grid, field.distances_m, field.occupancy_fingerprint});
  if (!upload.accepted) {
    return;
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
  prepared.producer_instance_id = raw_world.producer_instance_id;
  prepared.revision = field.occupancy_fingerprint;
  prepared.source_occupied_fingerprint = field.occupancy_fingerprint;
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
  prepared.observed_occupancy = field.local_occupancy;
  prepared.proprioceptive_free_space_seed = free_space_seed;
  prepared.launch_support_contact = launch_support_contact_;
  prepared.launch_support_resolution_pending = launch_support_resolution_pending;
  prepared.topological_graph = topology_update.snapshot;
  prepared.topological_graph_update = topology_update.graph;
  if (const std::shared_ptr<const ProductionNavigationObjective> objective =
          navigationObjective()) {
    prepared.search_objective = makeStaticRouteObjective(*objective);
  }
  prepared.lattice_search_performed = false;
  prepared.lattice_continuation_attempt = 0U;

  {
    const std::scoped_lock lock{esdf_state_mutex_};
    prepared_esdf_ = prepared;
  }
  auto guide_world = std::make_shared<const ProductionMppiPreparedEsdf>(prepared);
  {
    const std::scoped_lock lock{guide_queue_mutex_};
    if (pending_guide_world_) {
      dropped_guide_worlds_.fetch_add(1U, std::memory_order_relaxed);
    }
    pending_guide_world_ = std::move(guide_world);
  }
  guide_queue_condition_.notify_all();
  if (!world_ready_.exchange(true, std::memory_order_acq_rel)) {
    publishWorldReadiness(true);
  }

  RCLCPP_INFO(get_logger(),
              "PRODUCTION_MPPI_ESDF3D_ONLINE revision=%" PRIu64 " raw_revision=%" PRIu64
              " build_ms=%.2f classify_ms=%.2f upload_ms=%.2f dimensions=%dx%dx%d "
              "known=%zu free=%zu occupied=%zu unknown=%zu proprioceptive_free=%zu "
              "launch_support=%zu "
              "recenter=%s builds=%" PRIu64 " throttled=%" PRIu64
              " dropped_raw=%" PRIu64,
              prepared.revision, raw_world.revision, prepared.build_ms,
              field.stats.classification_ms, prepared.upload_ms, prepared.grid.width,
              prepared.grid.height, prepared.grid.depth, field.stats.known_voxels,
              field.stats.free_voxels, field.stats.occupied_voxels,
              field.stats.unknown_voxels, field.stats.proprioceptive_free_voxels,
              field.stats.launch_support_voxels, recenter ? "true" : "false",
              no_static_esdf_builds_.load(std::memory_order_relaxed),
              no_static_esdf_throttled_updates_.load(std::memory_order_relaxed),
              dropped_raw_snapshots_.load(std::memory_order_relaxed));
}

} // namespace drone_city_nav
