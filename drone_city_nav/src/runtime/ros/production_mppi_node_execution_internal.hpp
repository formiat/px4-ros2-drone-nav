#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>

#include "execution_horizon_assembler_3d.hpp"
#include "production_mppi_node.hpp"

namespace drone_city_nav {

enum class ProductionMppiHorizonCommitStatus : std::uint8_t {
  kPublished,
  kRejected,
};

namespace production_mppi_execution_detail {

[[nodiscard]] builtin_interfaces::msg::Time
timeFromNanoseconds(std::int64_t nanoseconds);

[[nodiscard]] std::int64_t
timeToNanoseconds(const builtin_interfaces::msg::Time& time) noexcept;

[[nodiscard]] std::optional<std::int64_t>
canonicalHorizonEndTime(std::int64_t valid_from_ns, std::int64_t duration_ns) noexcept;

[[nodiscard]] bool sameState(const mppi::State& first,
                             const mppi::State& second) noexcept;

[[nodiscard]] bool sameControl(const mppi::Control& first,
                               const mppi::Control& second) noexcept;

[[nodiscard]] LatestLidarEvidenceFreshness3D latestLidarEvidenceFreshness(
    const std::shared_ptr<const VersionedLatestLidarEvidence3D>& evidence,
    std::int64_t now_ns, double maximum_age_ms) noexcept;

[[nodiscard]] bool bindHorizonRouteMetadata(msg::MppiTrajectoryHorizon& horizon,
                                            const CertifiedRouteSuffix3D& route);

void appendStationaryHoldPoint(msg::MppiTrajectoryHorizon& horizon,
                               const Point3& hold_position,
                               std::int64_t time_from_start_ns, float yaw_rad);

// Encodes the states as horizon points whose acceleration is the acceleration
// the trajectory actually carries out: the command where the integrator let
// it through, the command plus the clamp's share where a speed cap shed the
// excess the command would have added.
[[nodiscard]] bool appendFiniteExecutionPoints(
    msg::MppiTrajectoryHorizon& horizon, std::span<const mppi::State> states,
    std::span<const mppi::Control> controls,
    const mppi::Control& previous_applied_control, std::int64_t control_interval_ns,
    const mppi::DynamicsConfig& dynamics);

} // namespace production_mppi_execution_detail

} // namespace drone_city_nav
