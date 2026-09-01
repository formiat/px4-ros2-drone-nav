#include "mppi_controller_3d.hpp"

#include <chrono>
#include <exception>
#include <utility>

namespace drone_city_nav {

bool MppiControllerWorldCurrentness3D::current() const noexcept {
  return status == MppiControllerWorldStatus3D::kCurrent;
}

MppiControllerWorldCurrentness3D assessMppiControllerWorldCurrentness3D(
    const WorldSnapshot3D& captured,
    const std::shared_ptr<const WorldSnapshot3D>& resident) noexcept {
  const ProductionWorldGenerationStatus captured_status =
      assessProductionWorldGeneration(captured);
  const ProductionWorldGenerationStatus resident_status =
      resident != nullptr ? assessProductionWorldGeneration(*resident)
                          : ProductionWorldGenerationStatus::kInvalidGeneration;
  if (resident == nullptr) {
    return {
        .status = MppiControllerWorldStatus3D::kMissingResident,
        .captured_generation_status = captured_status,
        .resident_generation_status = resident_status,
    };
  }
  if (captured_status != ProductionWorldGenerationStatus::kCoherent) {
    return {
        .status = MppiControllerWorldStatus3D::kInvalidCaptured,
        .captured_generation_status = captured_status,
        .resident_generation_status = resident_status,
    };
  }
  if (resident_status != ProductionWorldGenerationStatus::kCoherent) {
    return {
        .status = MppiControllerWorldStatus3D::kInvalidResident,
        .captured_generation_status = captured_status,
        .resident_generation_status = resident_status,
    };
  }
  return {
      .status =
          resident->local_world_generation.sameSnapshot(captured.local_world_generation)
              ? MppiControllerWorldStatus3D::kCurrent
              : MppiControllerWorldStatus3D::kSuperseded,
      .captured_generation_status = captured_status,
      .resident_generation_status = resident_status,
  };
}

const char* mppiControllerStatus3DName(const MppiControllerStatus3D status) noexcept {
  switch (status) {
    case MppiControllerStatus3D::kPlanned:
      return "planned";
    case MppiControllerStatus3D::kStationaryHold:
      return "stationary_hold";
    case MppiControllerStatus3D::kBackendUnavailable:
      return "backend_unavailable";
    case MppiControllerStatus3D::kBackendFailure:
      return "backend_failure";
  }
  return "unknown";
}

bool MppiControllerResult3D::executable() const noexcept {
  return status == MppiControllerStatus3D::kPlanned ||
         status == MppiControllerStatus3D::kStationaryHold;
}

MppiController3D::MppiController3D(const mppi::BenchmarkConfig& config)
    : engine_{config} {
}

bool MppiController3D::ready() const noexcept {
  const std::scoped_lock lock{mutex_};
  return engine_.ready();
}

mppi::EsdfUploadResult
MppiController3D::updateEsdf(const mppi::EsdfSnapshot& snapshot) {
  const std::scoped_lock lock{mutex_};
  return engine_.updateEsdf(snapshot);
}

MppiControllerResult3D MppiController3D::run(MppiControllerRequest3D request) {
  const std::scoped_lock lock{mutex_};
  const MppiNominalReseedUpdate nominal_reseed =
      nominal_reseed_tracker_.update(request.nominal_reseed);
  request.input.nominal_reseed_generation = nominal_reseed.generation;
  MppiControllerResult3D output{
      .status = MppiControllerStatus3D::kBackendUnavailable,
      .input = std::move(request.input),
      .result = {},
      .no_eligible_recovery =
          MppiEligibleRolloutUpdate{
              .no_eligible_recovery_generation =
                  nominal_reseed.no_eligible_recovery_generation,
              .phase = nominal_reseed.no_eligible_phase,
          },
      .failure_message = {},
  };

  if (request.mode == MppiControllerMode3D::kStationaryHold) {
    output.status = MppiControllerStatus3D::kStationaryHold;
    output.result.horizon = {output.input.target, output.input.target};
    output.result.controls = {mppi::Control{}};
    output.result.selected_tier = mppi::RiskTier::kPreferred;
    output.result.esdf_revision = request.world_revision;
    output.result.timings.host_total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  request.tick_started)
            .count();
    return output;
  }

  if (!engine_.ready()) {
    output.status = MppiControllerStatus3D::kBackendUnavailable;
    return output;
  }

  try {
    output.result = engine_.plan(output.input);
  } catch (const std::exception& error) {
    output.status = MppiControllerStatus3D::kBackendFailure;
    output.failure_message = error.what();
    return output;
  } catch (...) {
    output.status = MppiControllerStatus3D::kBackendFailure;
    output.failure_message = "unknown exception";
    return output;
  }

  output.status = MppiControllerStatus3D::kPlanned;
  output.no_eligible_recovery = nominal_reseed_tracker_.observeEligibleRolloutResult(
      output.result.feasibility_contract.available, output.result.nominal_reseeded);
  return output;
}

} // namespace drone_city_nav
