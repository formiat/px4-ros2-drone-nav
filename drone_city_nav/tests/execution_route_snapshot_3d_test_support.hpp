#pragma once

#include "drone_city_nav/execution_route_store_3d.hpp"
#include "drone_city_nav/execution_route_transitions_3d.hpp"
#include "drone_city_nav/execution_supervisor_3d.hpp"
#include "drone_city_nav/mppi/mppi_reference.hpp"
#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/trajectory_compiler_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <barrier>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "compiled_trajectory_3d_test_support.hpp"

namespace drone_city_nav {
namespace {

struct SnapshotFixture3D {
  static constexpr std::uint64_t kRouteGeneration{1U};
  static constexpr std::uint64_t kRawProducer{9U};
  static constexpr std::uint64_t kLatestRawRevision{18U};

  std::vector<RouteSample3D> route{
      RouteSample3D{.position = {0.0, 0.0, 5.0},
                    .tangent = {1.0, 0.0, 0.0},
                    .station_m = 0.0,
                    .reference_speed_mps = 4.0},
      RouteSample3D{.position = {5.0, 0.0, 5.0},
                    .tangent = {1.0, 0.0, 0.0},
                    .station_m = 5.0,
                    .reference_speed_mps = 4.0},
      RouteSample3D{.position = {10.0, 0.0, 5.0},
                    .tangent = {1.0, 0.0, 0.0},
                    .station_m = 10.0,
                    .reference_speed_mps = 0.0},
  };
  std::uint64_t physical_route_fingerprint{routeFingerprint(route)};
  NavigationWorldCertificate3D planned_world{
      .producer_instance_id = kRawProducer,
      .esdf_fingerprint = 70U,
      .esdf_source_raw_revision = 12U,
      .esdf_source_occupied_fingerprint = 21U,
      .raw_validated_through_revision = 12U,
      .local_world_generation = 3U,
      .topology_revision = 2U,
  };
  NavigationWorldCertificate3D validated_world{
      .producer_instance_id = kRawProducer,
      .esdf_fingerprint = 72U,
      .esdf_source_raw_revision = 14U,
      .esdf_source_occupied_fingerprint = 23U,
      .raw_validated_through_revision = 14U,
      .local_world_generation = 4U,
      .topology_revision = 3U,
  };
  StaticRouteObjective objective{
      .goal = {10.0, 0.0, 5.0},
      .mission_epoch = 3U,
      .sample_sequence = 6U,
      .assignment_generation = 4U,
      .available = true,
  };
  MaterializedRouteProposal3D proposal{
      .planned_world = planned_world,
      .validated_world = validated_world,
      .objective = objective,
      .intent =
          RouteIntent3D{
              .id = 44U,
              .planned_on_revision = 12U,
              .mission_target = {10.0, 0.0, 5.0},
              .valid = true,
          },
      .evidence =
          SegmentEvidence3D{
              .planned_on_revision = 12U,
              .validated_through_revision = 14U,
              .status = SegmentEvidenceStatus3D::kValid,
              .route_length_m = 10.0,
              .endpoint_displacement_m = 8.0,
              .materialized = true,
              .planner_executable = true,
              .physical_executable = true,
              .reaches_mission_target = true,
          },
      .route_fingerprint = physical_route_fingerprint,
      .route_sample_count = route.size(),
      .reaches_mission_goal = true,
      .activation_eligible = true,
  };
  ObservedOccupancyGrid3D raw_occupancy{GridBounds3D{-5.0, -5.0, 0.0, 1.0, 20, 10, 10}};
  std::shared_ptr<const CompiledTrajectory3D> geometry{
      makeGeometry(route, physical_route_fingerprint,
                   TrackingErrorTubeWorld3D{
                       .observed_occupancy = &raw_occupancy,
                       .occupied_content_fingerprint =
                           raw_occupancy.occupiedSnapshot().contentFingerprint(),
                   })};
  std::uint64_t geometry_revision{geometry->compiled_trajectory_revision};
  PassageVolumeConfig passage_volume_config{testPassageVolumeConfig()};
  SweptFootprintConfig execution_footprint{testPassageVolumeConfig().footprint};
  std::shared_ptr<const VersionedExecutionValidationPolicy3D> validation_policy = [] {
    mppi::DynamicsConfig dynamics;
    dynamics.dt_s = 0.1F;
    dynamics.linear_drag_1ps = 0.0F;
    return VersionedExecutionValidationPolicy3D::capture(
        FlightEnvelopeConfig{}, dynamics, mppi::AltitudeEnvelopeConfig{},
        testPassageVolumeConfig().footprint, 100.0, 1000.0, 1000.0, true);
  }();

  [[nodiscard]] RouteActivationObservation3D observation() const noexcept {
    return RouteActivationObservation3D{
        .resident_world = validated_world,
        .current_objective = objective,
        .minimum_tracking_sample_sequence = objective.sample_sequence,
        .position = {2.0, 0.0, 5.0},
        .maximum_cross_track_m = 2.0,
        .latest_raw_occupancy = &raw_occupancy,
        .latest_raw_producer_instance_id = kRawProducer,
        .latest_raw_revision = kLatestRawRevision,
        .footprint = execution_footprint,
        .raw_validation_required = true,
    };
  }

  [[nodiscard]] ExecutionRouteActivation3D activation() const {
    const std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world =
        VersionedObservedRawWorld3D::capture(
            RawMapVersion{
                .producer_instance_id = kRawProducer,
                .base_snapshot_revision = 0U,
                .revision = kLatestRawRevision,
            },
            raw_occupancy, std::nullopt, std::nullopt);
    return ExecutionRouteActivation3D{
        .route_generation = kRouteGeneration,
        .proposal = proposal,
        .geometry = geometry,
        .observation = observation(),
        .passage_volume_config = passage_volume_config,
        .continuity_lineage =
            RouteContinuityLineage3D{
                .mission_epoch = objective.mission_epoch,
                .assignment_generation = objective.assignment_generation,
            },
        .observed_raw_world = observed_raw_world,
        .static_world = nullptr,
        .validation_policy = validation_policy,
        .retained_route_owner = std::nullopt,
    };
  }

  [[nodiscard]] RouteExecutionObservation3D
  executionObservation(const Point3& position, const std::uint64_t raw_revision,
                       const ObservedOccupancyGrid3D* const occupancy) const noexcept {
    const RouteActivationObservation3D activation_observation = observation();
    return RouteExecutionObservation3D{
        .current_objective = objective,
        .minimum_tracking_sample_sequence = objective.sample_sequence,
        .position = position,
        .maximum_cross_track_m = 2.0,
        .latest_raw_occupancy = occupancy,
        .latest_raw_producer_instance_id = kRawProducer,
        .latest_raw_revision = raw_revision,
        .footprint = activation_observation.footprint,
    };
  }

  [[nodiscard]] std::shared_ptr<const VersionedObservedRawWorld3D>
  rawWorld(const std::uint64_t revision,
           const ObservedOccupancyGrid3D* const occupancy = nullptr,
           const std::uint64_t producer_instance_id = kRawProducer) const {
    return VersionedObservedRawWorld3D::capture(
        RawMapVersion{
            .producer_instance_id = producer_instance_id,
            .base_snapshot_revision = 0U,
            .revision = revision,
        },
        occupancy != nullptr ? *occupancy : raw_occupancy, std::nullopt, std::nullopt);
  }

  [[nodiscard]] std::optional<CertifiedRouteSuffix3D> certify() const {
    return certifyExecutionRoute3D(activation());
  }

  [[nodiscard]] std::shared_ptr<const ExecutionPlan3D> activeSnapshot() const {
    const std::optional<CertifiedRouteSuffix3D> suffix = certify();
    if (!suffix.has_value()) {
      return nullptr;
    }
    const std::shared_ptr<const ExecutionPlan3D> initial =
        makeInitialExecutionRouteSnapshot3D();
    if (!initial) {
      return nullptr;
    }
    FiniteExecutionPlan3D initial_execution =
        finitePlanForRoute(*initial, *suffix, FiniteExecutionKind3D::kNominal, 100U);
    return activateCertifiedRoute3D(*initial, initial->version, *suffix,
                                    std::move(initial_execution))
        .next;
  }

  [[nodiscard]] static ExecutionRouteTransitionGuard3D
  guard(const ExecutionPlan3D& snapshot) noexcept {
    const CertifiedRouteSuffix3D* const route = snapshot.route();
    if (route == nullptr || route->geometry == nullptr) {
      return {};
    }
    return ExecutionRouteTransitionGuard3D{
        .expected_snapshot_version = snapshot.version,
        .expected_route_generation = route->identity.generation,
        .expected_geometry_revision = route->geometry->compiled_trajectory_revision,
    };
  }

  [[nodiscard]] static std::shared_ptr<const VersionedExecutionInput3D>
  progressInput(const ExecutionPlan3D& snapshot, const Point3& position,
                const std::uint64_t identity_increment = 1U) {
    const CertifiedRouteSuffix3D* const route = snapshot.route();
    if (route == nullptr || route->progress.execution_input == nullptr ||
        identity_increment == 0U) {
      return nullptr;
    }
    const VersionedExecutionInput3D& previous = *route->progress.execution_input;
    const auto advance_unsigned = [identity_increment](const std::uint64_t value) {
      if (value > std::numeric_limits<std::uint64_t>::max() - identity_increment) {
        throw std::overflow_error{"execution input fixture identity overflow"};
      }
      return value + identity_increment;
    };
    constexpr std::int64_t kIdentityStepNs{10'000LL};
    if (identity_increment >
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max() /
                                   kIdentityStepNs)) {
      throw std::overflow_error{"execution input fixture timestamp overflow"};
    }
    const std::int64_t stamp_delta_ns =
        static_cast<std::int64_t>(identity_increment) * kIdentityStepNs;
    const auto advance_stamp = [stamp_delta_ns](const std::int64_t value) {
      if (value > std::numeric_limits<std::int64_t>::max() - stamp_delta_ns) {
        throw std::overflow_error{"execution input fixture timestamp overflow"};
      }
      return value + stamp_delta_ns;
    };
    mppi::State state = previous.state();
    state.x = static_cast<float>(position.x);
    state.y = static_cast<float>(position.y);
    state.z = static_cast<float>(position.z);
    return VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
        .capture_sequence = advance_unsigned(previous.captureSequence()),
        .pose_revision = advance_unsigned(previous.poseRevision()),
        .pose_source_timestamp_us = advance_unsigned(previous.poseSourceTimestampUs()),
        .pose_receive_stamp_ns = advance_stamp(previous.poseReceiveStampNs()),
        .effective_stamp_ns = previous.effectiveStampNs(),
        .state = state,
        .full_state_authoritative = previous.fullStateAuthoritative(),
        .state_provenance = previous.stateProvenance(),
        .previous_control = previous.previousControl(),
        .previous_control_source = previous.previousControlSource(),
        .previous_control_source_producer_instance_id =
            previous.previousControlSourceProducerInstanceId(),
        .previous_control_source_sequence =
            advance_unsigned(previous.previousControlSourceSequence()),
        .previous_control_source_stamp_ns =
            advance_stamp(previous.previousControlSourceStampNs()),
        .previous_control_receive_stamp_ns =
            advance_stamp(previous.previousControlReceiveStampNs()),
    });
  }

  [[nodiscard]] static FiniteExecutionCertification3D finiteCertificationForRoute(
      const CertifiedRouteSuffix3D& suffix,
      const FiniteExecutionKind3D kind = FiniteExecutionKind3D::kNominal,
      const std::uint64_t trajectory_revision = 101U,
      const std::uint64_t source_navigation_revision = 55U,
      const std::size_t extra_stationary_control_count = 0U,
      const double requested_begin_station_m = -1.0,
      const VersionedExecutionInput3D* const minimum_execution_input = nullptr,
      const VersionedLatestLidarEvidence3D* const minimum_lidar_evidence = nullptr) {
    const double begin_station_m = requested_begin_station_m >= 0.0
                                       ? requested_begin_station_m
                                       : suffix.progress.station_m;
    const RouteSample3D begin_sample =
        sampleRoute3DAtStation(*suffix.geometry->route, begin_station_m);
    if (suffix.validation_policy == nullptr || !std::isfinite(begin_station_m) ||
        begin_station_m < suffix.progress.station_m ||
        begin_station_m > suffix.endStationM()) {
      throw std::logic_error{"finite execution fixture requires a policy"};
    }
    const mppi::DynamicsConfig& dynamics = suffix.validation_policy->dynamics();
    constexpr std::size_t kAccelerationControlCount{50U};
    const std::size_t planned_control_count =
        2U * kAccelerationControlCount + 1U + extra_stationary_control_count;
    std::vector<mppi::State> planned_states(planned_control_count + 1U);
    planned_states.front() =
        mppi::State{.x = static_cast<float>(begin_sample.position.x),
                    .y = static_cast<float>(begin_sample.position.y),
                    .z = static_cast<float>(begin_sample.position.z)};
    const float acceleration_mps2 =
        static_cast<float>((suffix.endStationM() - begin_station_m) / 25.0);
    std::vector<mppi::Control> planned_controls(planned_control_count);
    for (std::size_t index = 0U; index < kAccelerationControlCount; ++index) {
      planned_controls[index].ax = acceleration_mps2;
    }
    for (std::size_t index = kAccelerationControlCount;
         index < 2U * kAccelerationControlCount; ++index) {
      planned_controls[index].ax = -acceleration_mps2;
    }
    for (std::size_t index = 0U; index < planned_controls.size(); ++index) {
      planned_states[index + 1U] = mppi::integrateReference(
          planned_states[index], planned_controls[index], dynamics);
    }
    std::optional<mppi::FiniteHorizon> built_horizon =
        mppi::buildFiniteHorizon(planned_states, planned_controls,
                                 planned_controls.size(), dynamics, mppi::Control{});
    if (!built_horizon.has_value()) {
      throw std::logic_error{"failed to build finite execution fixture"};
    }
    mppi::FiniteHorizon horizon = std::move(*built_horizon);
    if (kind == FiniteExecutionKind3D::kEmergencyBrakeTail) {
      built_horizon = mppi::buildFiniteBrakingHorizon(
          horizon.states.front(), horizon.controls.size(), dynamics, mppi::Control{});
      if (!built_horizon.has_value()) {
        throw std::logic_error{"failed to build braking execution fixture"};
      }
      horizon = std::move(*built_horizon);
    }
    const VersionedExecutionInput3D* const progress_input =
        suffix.progress.execution_input != nullptr
            ? suffix.progress.execution_input.get()
            : minimum_execution_input;
    const auto next_identity = [](const std::uint64_t value) {
      if (value == std::numeric_limits<std::uint64_t>::max()) {
        throw std::overflow_error{"execution input fixture identity overflow"};
      }
      return value + 1U;
    };
    const auto next_stamp = [](const std::int64_t value) {
      constexpr std::int64_t kEvidenceStepNs{10'000LL};
      if (value > std::numeric_limits<std::int64_t>::max() - kEvidenceStepNs) {
        throw std::overflow_error{"execution input fixture timestamp overflow"};
      }
      return value + kEvidenceStepNs;
    };
    const bool effective_stamp_advance_required =
        progress_input != nullptr &&
        (progress_input->poseReceiveStampNs() == progress_input->effectiveStampNs() ||
         progress_input->previousControlSourceStampNs() ==
             progress_input->effectiveStampNs() ||
         progress_input->previousControlReceiveStampNs() ==
             progress_input->effectiveStampNs());
    const std::int64_t valid_from_ns =
        progress_input == nullptr ? 1'000'000'000LL
        : effective_stamp_advance_required
            ? next_stamp(progress_input->effectiveStampNs())
            : progress_input->effectiveStampNs();
    const std::int64_t evidence_offset_ns =
        static_cast<std::int64_t>(trajectory_revision) * 10'000LL;
    const std::uint64_t capture_sequence =
        progress_input != nullptr
            ? std::max(trajectory_revision,
                       next_identity(progress_input->captureSequence()))
            : trajectory_revision;
    const std::uint64_t pose_revision =
        progress_input != nullptr
            ? std::max(source_navigation_revision,
                       next_identity(progress_input->poseRevision()))
            : source_navigation_revision;
    const std::uint64_t pose_source_timestamp_us =
        progress_input != nullptr
            ? std::max(900'000U + source_navigation_revision,
                       next_identity(progress_input->poseSourceTimestampUs()))
            : 900'000U + source_navigation_revision;
    const std::int64_t pose_receive_stamp_ns =
        progress_input != nullptr
            ? std::max<std::int64_t>(valid_from_ns - 20'000'000LL + evidence_offset_ns,
                                     next_stamp(progress_input->poseReceiveStampNs()))
            : valid_from_ns - 20'000'000LL + evidence_offset_ns;
    const std::uint64_t control_source_sequence =
        progress_input != nullptr
            ? std::max(trajectory_revision,
                       next_identity(progress_input->previousControlSourceSequence()))
            : trajectory_revision;
    const std::int64_t control_source_stamp_ns =
        progress_input != nullptr
            ? std::max<std::int64_t>(
                  valid_from_ns - 15'000'000LL + evidence_offset_ns,
                  next_stamp(progress_input->previousControlSourceStampNs()))
            : valid_from_ns - 15'000'000LL + evidence_offset_ns;
    const std::int64_t control_receive_stamp_ns =
        progress_input != nullptr
            ? std::max<std::int64_t>(
                  valid_from_ns - 10'000'000LL + evidence_offset_ns,
                  next_stamp(progress_input->previousControlReceiveStampNs()))
            : valid_from_ns - 10'000'000LL + evidence_offset_ns;
    const ExecutionStateProvenance3D state_provenance{
        .x = ExecutionStateFieldProvenance3D::kSourceSample,
        .y = ExecutionStateFieldProvenance3D::kSourceSample,
        .z = ExecutionStateFieldProvenance3D::kSourceSample,
        .vx = ExecutionStateFieldProvenance3D::kSourceSample,
        .vy = ExecutionStateFieldProvenance3D::kSourceSample,
        .vz = ExecutionStateFieldProvenance3D::kSourceSample,
        .yaw = ExecutionStateFieldProvenance3D::kSourceSample,
        .yaw_rate = ExecutionStateFieldProvenance3D::kSourceSample,
    };
    const std::shared_ptr<const VersionedExecutionInput3D> execution_input =
        VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
            .capture_sequence = capture_sequence,
            .pose_revision = pose_revision,
            .pose_source_timestamp_us = pose_source_timestamp_us,
            .pose_receive_stamp_ns = pose_receive_stamp_ns,
            .effective_stamp_ns = valid_from_ns,
            .state = horizon.states.front(),
            .full_state_authoritative = true,
            .state_provenance = state_provenance,
            .previous_control = {},
            .previous_control_source =
                ExecutionPreviousControlEvidenceSource3D::kMeasuredAcceleration,
            .previous_control_source_sequence = control_source_sequence,
            .previous_control_source_stamp_ns = control_source_stamp_ns,
            .previous_control_receive_stamp_ns = control_receive_stamp_ns,
        });
    const std::uint64_t lidar_sequence =
        minimum_lidar_evidence != nullptr
            ? std::max(trajectory_revision,
                       next_identity(minimum_lidar_evidence->sequence()))
            : trajectory_revision;
    const std::uint64_t lidar_pose_generation =
        minimum_lidar_evidence != nullptr
            ? std::max(pose_revision,
                       next_identity(minimum_lidar_evidence->poseGeneration()))
            : pose_revision;
    const std::int64_t lidar_acquisition_stamp_ns =
        minimum_lidar_evidence != nullptr
            ? std::max<std::int64_t>(
                  valid_from_ns - std::int64_t{15'000'000} + evidence_offset_ns,
                  next_stamp(minimum_lidar_evidence->acquisitionStampNs()))
            : valid_from_ns - 15'000'000LL + evidence_offset_ns;
    const std::int64_t lidar_receive_stamp_ns =
        minimum_lidar_evidence != nullptr
            ? std::max<std::int64_t>(
                  valid_from_ns - std::int64_t{5'000'000} + evidence_offset_ns,
                  next_stamp(minimum_lidar_evidence->receiveStampNs()))
            : valid_from_ns - 5'000'000LL + evidence_offset_ns;
    const std::shared_ptr<const VersionedLatestLidarEvidence3D> lidar_evidence =
        VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
            .producer_instance_id = minimum_lidar_evidence != nullptr
                                        ? minimum_lidar_evidence->producerInstanceId()
                                        : 77U,
            .sequence = lidar_sequence,
            .pose_generation = lidar_pose_generation,
            .acquisition_stamp_ns = lidar_acquisition_stamp_ns,
            .receive_stamp_ns = lidar_receive_stamp_ns,
            .source_beam_count = 1U,
            .hit_points_map_m = {},
        });
    if (execution_input == nullptr) {
      throw std::logic_error{"failed to capture finite execution input"};
    }
    if (lidar_evidence == nullptr) {
      throw std::logic_error{"failed to capture finite execution lidar evidence"};
    }
    return FiniteExecutionCertification3D{
        .trajectory_revision = trajectory_revision,
        .horizon = std::move(horizon),
        .execution_input = execution_input,
        .latest_lidar_evidence = lidar_evidence,
        .valid_from_ns = valid_from_ns,
        .kind = kind,
    };
  }

  [[nodiscard]] static FiniteExecutionState3D finiteExecutionForRoute(
      const ExecutionPlan3D& snapshot, const CertifiedRouteSuffix3D& suffix,
      const FiniteExecutionKind3D kind = FiniteExecutionKind3D::kNominal,
      const bool terminal_rest = true, const std::uint64_t trajectory_revision = 101U,
      const std::uint64_t source_navigation_revision = 55U,
      const std::size_t extra_stationary_control_count = 0U,
      const double begin_station_m = -1.0) {
    const CertifiedRouteSuffix3D* const resident_route = snapshot.route();
    const StationaryExecutionHold3D* const resident_hold = snapshot.stationaryHold();
    const DirectTrackingFiniteExecution3D* const direct_execution =
        snapshot.directTrackingExecution();
    const FiniteExecutionState3D* const finite_execution = snapshot.finiteExecution();
    const VersionedExecutionInput3D* const minimum_execution_input =
        resident_route != nullptr     ? resident_route->progress.execution_input.get()
        : resident_hold != nullptr    ? resident_hold->terminal_execution_input.get()
        : direct_execution != nullptr ? direct_execution->execution_input.get()
                                      : nullptr;
    const VersionedLatestLidarEvidence3D* const minimum_lidar_evidence =
        finite_execution != nullptr   ? finite_execution->latest_lidar_evidence.get()
        : resident_hold != nullptr    ? resident_hold->latest_lidar_evidence.get()
        : direct_execution != nullptr ? direct_execution->latest_lidar_evidence.get()
                                      : nullptr;
    const std::optional<FiniteExecutionState3D> certified = certifyFiniteExecution3D(
        snapshot, suffix,
        finiteCertificationForRoute(suffix, kind, trajectory_revision,
                                    source_navigation_revision,
                                    extra_stationary_control_count, begin_station_m,
                                    minimum_execution_input, minimum_lidar_evidence));
    if (!certified.has_value()) {
      throw std::logic_error{"valid finite execution fixture was rejected"};
    }
    FiniteExecutionState3D result = certified.value();
    if (!terminal_rest) {
      auto changed_horizon = std::make_shared<mppi::FiniteHorizon>(*result.horizon);
      changed_horizon->states.back().vx = 1.0F;
      result.horizon = std::move(changed_horizon);
    }
    return result;
  }

  [[nodiscard]] static FiniteExecutionPlan3D finitePlanForRoute(
      const ExecutionPlan3D& snapshot, const CertifiedRouteSuffix3D& suffix,
      const FiniteExecutionKind3D command_kind = FiniteExecutionKind3D::kNominal,
      const std::uint64_t trajectory_revision = 101U,
      const std::uint64_t source_navigation_revision = 55U,
      const std::size_t extra_stationary_control_count = 0U,
      const double begin_station_m = -1.0);

  [[nodiscard]] static FiniteExecutionState3D rawInvalidatedFiniteExecution(
      const ExecutionPlan3D& snapshot, const RouteLifecycleEvent3D& invalidation,
      std::shared_ptr<const VersionedObservedRawWorld3D> invalidating_world,
      const FiniteExecutionKind3D kind = FiniteExecutionKind3D::kEmergencyBrakeTail,
      const bool terminal_rest = true, const std::uint64_t trajectory_revision = 101U,
      const std::uint64_t source_navigation_revision = 55U,
      const std::size_t extra_stationary_control_count = 0U,
      const double begin_station_m = -1.0) {
    const CertifiedRouteSuffix3D* const route = snapshot.route();
    if (route == nullptr) {
      throw std::logic_error{"raw revalidation fixture requires a route"};
    }
    const std::optional<FiniteExecutionState3D> certified =
        certifyRawInvalidatedFiniteExecution3D(
            snapshot,
            RawInvalidatedFiniteExecutionCertification3D{
                .invalidation = invalidation,
                .invalidating_observed_raw_world = std::move(invalidating_world),
                .finite_execution = finiteCertificationForRoute(
                    *route, kind, trajectory_revision, source_navigation_revision,
                    extra_stationary_control_count, begin_station_m),
            });
    if (!certified.has_value()) {
      throw std::logic_error{"valid raw revalidation fixture was rejected"};
    }
    FiniteExecutionState3D result = certified.value();
    if (!terminal_rest) {
      auto changed_horizon = std::make_shared<mppi::FiniteHorizon>(*result.horizon);
      changed_horizon->states.back().vx = 1.0F;
      result.horizon = std::move(changed_horizon);
    }
    return result;
  }

  [[nodiscard]] static FiniteExecutionState3D
  finiteExecution(const ExecutionPlan3D& snapshot,
                  const FiniteExecutionKind3D kind = FiniteExecutionKind3D::kNominal,
                  const bool terminal_rest = true,
                  const std::uint64_t trajectory_revision = 101U,
                  const std::uint64_t source_navigation_revision = 55U,
                  const std::size_t extra_stationary_control_count = 0U,
                  const double begin_station_m = -1.0) {
    const CertifiedRouteSuffix3D* const route = snapshot.route();
    if (route == nullptr) {
      throw std::logic_error{"finite execution fixture requires a route"};
    }
    return finiteExecutionForRoute(snapshot, *route, kind, terminal_rest,
                                   trajectory_revision, source_navigation_revision,
                                   extra_stationary_control_count, begin_station_m);
  }

  [[nodiscard]] static StationaryExecutionHoldCertification3D holdCertification(
      const ExecutionPlan3D& snapshot, const bool terminal_rest = true,
      const std::optional<Point3> requested_position = std::nullopt,
      const std::optional<std::int64_t> requested_effective_stamp_ns = std::nullopt,
      const std::optional<float> residual_velocity_x_mps = std::nullopt,
      const std::optional<mppi::Control> measured_control = std::nullopt) {
    const FiniteExecutionState3D* const route_execution = snapshot.finiteExecution();
    const DirectTrackingFiniteExecution3D* const direct_execution =
        snapshot.directTrackingExecution();
    const StationaryExecutionHold3D* const resident_hold = snapshot.stationaryHold();
    const std::shared_ptr<const VersionedExecutionInput3D> source_input =
        route_execution != nullptr    ? route_execution->execution_input
        : direct_execution != nullptr ? direct_execution->execution_input
        : resident_hold != nullptr    ? resident_hold->terminal_execution_input
                                      : nullptr;
    const mppi::FiniteHorizon* const source_horizon =
        route_execution != nullptr    ? route_execution->horizon.get()
        : direct_execution != nullptr ? direct_execution->horizon.get()
                                      : nullptr;
    if (source_input == nullptr ||
        (source_horizon == nullptr && resident_hold == nullptr)) {
      throw std::logic_error{"hold fixture requires an execution owner"};
    }
    const std::int64_t source_valid_until_ns =
        route_execution != nullptr    ? route_execution->valid_until_ns
        : direct_execution != nullptr ? direct_execution->valid_until_ns
                                      : source_input->effectiveStampNs();
    if (source_input->captureSequence() == std::numeric_limits<std::uint64_t>::max() ||
        source_input->poseRevision() == std::numeric_limits<std::uint64_t>::max() ||
        source_input->poseSourceTimestampUs() ==
            std::numeric_limits<std::uint64_t>::max() ||
        source_input->previousControlSourceSequence() ==
            std::numeric_limits<std::uint64_t>::max() ||
        source_valid_until_ns >
            std::numeric_limits<std::int64_t>::max() - std::int64_t{100'000}) {
      throw std::overflow_error{"hold fixture identity overflow"};
    }
    const std::int64_t effective_stamp_ns = requested_effective_stamp_ns.value_or(
        std::max(source_valid_until_ns,
                 source_input->effectiveStampNs() + std::int64_t{100'000}));
    if (effective_stamp_ns <= source_input->effectiveStampNs() + std::int64_t{30'000}) {
      throw std::logic_error{"hold fixture requires newer effective time"};
    }
    mppi::State state = resident_hold != nullptr ? source_input->state()
                                                 : source_horizon->states.back();
    if (!terminal_rest) {
      state.vx = 1.0F;
    } else if (residual_velocity_x_mps.has_value()) {
      state.vx = *residual_velocity_x_mps;
    }
    const Point3 terminal_position{state.x, state.y, state.z};
    const ExecutionStateProvenance3D state_provenance{
        .x = ExecutionStateFieldProvenance3D::kSourceSample,
        .y = ExecutionStateFieldProvenance3D::kSourceSample,
        .z = ExecutionStateFieldProvenance3D::kSourceSample,
        .vx = ExecutionStateFieldProvenance3D::kSourceSample,
        .vy = ExecutionStateFieldProvenance3D::kSourceSample,
        .vz = ExecutionStateFieldProvenance3D::kSourceSample,
        .yaw = ExecutionStateFieldProvenance3D::kSourceSample,
        .yaw_rate = ExecutionStateFieldProvenance3D::kSourceSample,
    };
    const std::shared_ptr<const VersionedExecutionInput3D> terminal_input =
        VersionedExecutionInput3D::capture(ExecutionInputCapture3D{
            .capture_sequence = source_input->captureSequence() + 1U,
            .pose_revision = source_input->poseRevision() + 1U,
            .pose_source_timestamp_us = source_input->poseSourceTimestampUs() + 1U,
            .pose_receive_stamp_ns = effective_stamp_ns - 30'000LL,
            .effective_stamp_ns = effective_stamp_ns,
            .state = state,
            .full_state_authoritative = true,
            .state_provenance = state_provenance,
            .previous_control = measured_control.value_or(mppi::Control{}),
            .previous_control_source = source_input->previousControlSource(),
            .previous_control_source_producer_instance_id =
                source_input->previousControlSourceProducerInstanceId(),
            .previous_control_source_sequence =
                source_input->previousControlSourceSequence() + 1U,
            .previous_control_source_stamp_ns = effective_stamp_ns - 20'000LL,
            .previous_control_receive_stamp_ns = effective_stamp_ns - 10'000LL,
        });
    if (terminal_input == nullptr) {
      throw std::logic_error{"failed to capture terminal hold input"};
    }
    const std::shared_ptr<const VersionedLatestLidarEvidence3D> source_lidar =
        resident_hold != nullptr     ? resident_hold->latest_lidar_evidence
        : route_execution != nullptr ? route_execution->latest_lidar_evidence
                                     : direct_execution->latest_lidar_evidence;
    if (source_lidar == nullptr ||
        source_lidar->sequence() == std::numeric_limits<std::uint64_t>::max() ||
        source_lidar->poseGeneration() == std::numeric_limits<std::uint64_t>::max()) {
      throw std::overflow_error{"hold fixture lidar identity overflow"};
    }
    const std::shared_ptr<const VersionedLatestLidarEvidence3D> current_lidar =
        resident_hold != nullptr
            ? source_lidar
            : VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
                  .producer_instance_id = source_lidar->producerInstanceId(),
                  .sequence = source_lidar->sequence() + 1U,
                  .pose_generation = source_lidar->poseGeneration() + 1U,
                  .acquisition_stamp_ns = effective_stamp_ns - 20'000LL,
                  .receive_stamp_ns = effective_stamp_ns - 10'000LL,
                  .source_beam_count = source_lidar->sourceBeamCount(),
                  .invalid_beam_count = source_lidar->invalidBeamCount(),
                  .hit_points_map_m = source_lidar->hitPointsMapM(),
              });
    if (current_lidar == nullptr) {
      throw std::logic_error{"failed to capture current hold lidar"};
    }
    return StationaryExecutionHoldCertification3D{
        .position = requested_position.value_or(terminal_position),
        .execution_input = terminal_input,
        .observed_raw_world =
            resident_hold != nullptr     ? resident_hold->observed_raw_world
            : route_execution != nullptr ? route_execution->observed_raw_world
                                         : direct_execution->observed_raw_world,
        .static_world = resident_hold != nullptr     ? resident_hold->static_world
                        : route_execution != nullptr ? route_execution->static_world
                                                     : direct_execution->static_world,
        .validation_policy = resident_hold != nullptr ? resident_hold->validation_policy
                             : route_execution != nullptr
                                 ? route_execution->validation_policy
                                 : direct_execution->validation_policy,
        .latest_lidar_evidence = current_lidar,
    };
  }

  [[nodiscard]] static std::shared_ptr<const VersionedLatestLidarEvidence3D>
  newerLidarEvidence(const VersionedLatestLidarEvidence3D& previous,
                     std::vector<Point3> hit_points_map_m = {}) {
    if (previous.sequence() == std::numeric_limits<std::uint64_t>::max() ||
        previous.poseGeneration() == std::numeric_limits<std::uint64_t>::max() ||
        previous.acquisitionStampNs() >
            std::numeric_limits<std::int64_t>::max() - 10'000LL ||
        previous.receiveStampNs() >
            std::numeric_limits<std::int64_t>::max() - 10'000LL) {
      throw std::overflow_error{"lidar fixture identity overflow"};
    }
    return VersionedLatestLidarEvidence3D::capture(LatestLidarEvidenceCapture3D{
        .producer_instance_id = previous.producerInstanceId(),
        .sequence = previous.sequence() + 1U,
        .pose_generation = previous.poseGeneration() + 1U,
        .acquisition_stamp_ns = previous.acquisitionStampNs() + 10'000LL,
        .receive_stamp_ns = previous.receiveStampNs() + 10'000LL,
        .source_beam_count = std::max<std::size_t>(1U, hit_points_map_m.size()),
        .hit_points_map_m = std::move(hit_points_map_m),
    });
  }

  [[nodiscard]] static std::shared_ptr<const VersionedExecutionInput3D>
  committedInput(const ExecutionPlan3D& plan) {
    if (const FiniteExecutionState3D* const execution = plan.finiteExecution()) {
      return execution->execution_input;
    }
    if (const DirectTrackingFiniteExecution3D* const execution =
            plan.directTrackingExecution()) {
      return execution->execution_input;
    }
    if (const StationaryExecutionHold3D* const hold = plan.stationaryHold()) {
      return hold->terminal_execution_input;
    }
    return nullptr;
  }

  [[nodiscard]] static ExecutionOwnerIdentity3D
  committedOwner(const ExecutionPlan3D& plan, const std::uint64_t sequence = 1U) {
    return ExecutionOwnerIdentity3D{
        .route_target = {10.0, 0.0, 5.0},
        .stationary_hold_position = {10.0, 0.0, 5.0},
        .valid_from_ns = 1'000'000'000LL,
        .valid_until_ns = 2'000'000'000LL,
        .producer_instance_id = 17U,
        .target_offboard_instance_id = 23U,
        .sequence = sequence,
        .execution_owner_epoch = plan.execution_owner_epoch,
        .execution_mode = ExecutionAuthorityMode3D::kPlanned,
        .execution_reason = ExecutionAuthorityReason3D::kNone,
        .valid = true,
    };
  }

  [[nodiscard]] static AppliedControlEvidence3D
  committedControl(const ExecutionOwnerIdentity3D& owner) {
    return AppliedControlEvidence3D{
        .source_stamp_ns = owner.valid_from_ns,
        .receive_stamp_ns = owner.valid_from_ns,
        .producer_instance_id = owner.target_offboard_instance_id,
        .horizon_producer_instance_id = owner.producer_instance_id,
        .horizon_sequence = owner.sequence,
        .content_fingerprint = owner.sequence,
        .execution_mode = owner.execution_mode,
        .yaw_acceleration_authoritative = true,
        .control_authoritative = true,
        .valid = true,
    };
  }
};

[[nodiscard, maybe_unused]] ExecutionRouteActivation3D
staticActivation(SnapshotFixture3D& fixture, const OccupancyGrid3D& static_occupancy) {
  ExecutionRouteActivation3D activation = fixture.activation();
  NavigationWorldCertificate3D planned_world = fixture.planned_world;
  NavigationWorldCertificate3D validated_world = fixture.validated_world;
  planned_world.producer_instance_id = 0U;
  planned_world.esdf_source_raw_revision = 0U;
  planned_world.esdf_source_occupied_fingerprint = 0U;
  planned_world.raw_validated_through_revision = 0U;
  validated_world.producer_instance_id = 0U;
  validated_world.esdf_source_raw_revision = 0U;
  validated_world.esdf_source_occupied_fingerprint = 0U;
  validated_world.raw_validated_through_revision = 0U;
  activation.proposal.planned_world = planned_world;
  activation.proposal.validated_world = validated_world;
  activation.observation.resident_world = validated_world;
  activation.observation.latest_raw_occupancy = nullptr;
  activation.observation.latest_raw_producer_instance_id = 0U;
  activation.observation.latest_raw_revision = 0U;
  activation.observation.raw_validation_required = false;
  activation.observed_raw_world.reset();
  activation.static_world =
      VersionedStaticWorld3D::capture(validated_world, static_occupancy);
  return activation;
}

[[nodiscard, maybe_unused]] std::optional<DirectTrackingFiniteExecution3D>
certifyDirectFixtureExecution(
    const ExecutionPlan3D& snapshot, CertifiedRouteSuffix3D path_route,
    const DirectTrackingOwnerIdentity3D& identity,
    const std::uint64_t trajectory_revision,
    const FiniteExecutionKind3D kind = FiniteExecutionKind3D::kNominal) {
  const DirectTrackingFiniteExecution3D* const previous_direct =
      snapshot.directTrackingExecution();
  const StationaryExecutionHold3D* const resident_hold = snapshot.stationaryHold();
  const FiniteExecutionState3D* const resident_execution = snapshot.finiteExecution();
  const VersionedExecutionInput3D* const minimum_execution_input =
      previous_direct != nullptr      ? previous_direct->execution_input.get()
      : resident_hold != nullptr      ? resident_hold->terminal_execution_input.get()
      : resident_execution != nullptr ? resident_execution->execution_input.get()
                                      : nullptr;
  const VersionedLatestLidarEvidence3D* const minimum_lidar_evidence =
      previous_direct != nullptr      ? previous_direct->latest_lidar_evidence.get()
      : resident_hold != nullptr      ? resident_hold->latest_lidar_evidence.get()
      : resident_execution != nullptr ? resident_execution->latest_lidar_evidence.get()
                                      : nullptr;
  if (previous_direct != nullptr) {
    path_route.progress.execution_input = previous_direct->execution_input;
    path_route.progress.last_observed_position =
        Point3{previous_direct->execution_input->state().x,
               previous_direct->execution_input->state().y,
               previous_direct->execution_input->state().z};
  } else if (resident_hold != nullptr) {
    path_route.progress.station_m = path_route.endStationM();
    path_route.progress.execution_input = resident_hold->terminal_execution_input;
    path_route.progress.last_observed_position = resident_hold->position;
  }
  FiniteExecutionCertification3D finite =
      SnapshotFixture3D::finiteCertificationForRoute(
          path_route, kind, trajectory_revision, trajectory_revision, 0U, -1.0,
          minimum_execution_input, minimum_lidar_evidence);
  return certifyDirectTrackingExecution3D(
      snapshot,
      DirectTrackingExecutionCertification3D{
          .identity = identity,
          .trajectory_revision = trajectory_revision,
          .target = {10.0, 0.0, 5.0},
          .horizon = std::move(finite.horizon),
          .observed_raw_world =
              previous_direct != nullptr ? previous_direct->observed_raw_world
              : resident_hold != nullptr ? resident_hold->observed_raw_world
                                         : path_route.observed_raw_world,
          .static_world = previous_direct != nullptr ? previous_direct->static_world
                          : resident_hold != nullptr ? resident_hold->static_world
                                                     : path_route.static_world,
          .validation_policy =
              previous_direct != nullptr ? previous_direct->validation_policy
              : resident_hold != nullptr ? resident_hold->validation_policy
                                         : path_route.validation_policy,
          .execution_input = std::move(finite.execution_input),
          .latest_lidar_evidence = std::move(finite.latest_lidar_evidence),
          .valid_from_ns = finite.valid_from_ns,
          .kind = kind,
      });
}

[[nodiscard, maybe_unused]] bool
publishPendingDraftForCurrentBase(RouteExecutionManager3D& manager,
                                  PendingCertifiedRoute3D candidate) {
  candidate.publication_sequence = 0U;
  return manager.publishPendingForCurrentBase(manager.plan(), std::move(candidate))
      .published();
}

[[nodiscard, maybe_unused]] bool
publishPendingDraftForCurrentBase(ExecutionSupervisor3D& supervisor,
                                  PendingCertifiedRoute3D candidate) {
  candidate.publication_sequence = 0U;
  return supervisor
      .publishPendingForCurrentBase(supervisor.plan(), std::move(candidate))
      .published();
}

} // namespace
} // namespace drone_city_nav
