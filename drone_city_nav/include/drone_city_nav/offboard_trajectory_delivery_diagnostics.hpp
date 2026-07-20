#pragma once

#include "drone_city_nav/trajectory_diagnostics_io.hpp"

#include <cstdint>
#include <string>

namespace drone_city_nav {

[[nodiscard]] bool
configFingerprintMismatch(std::uint64_t runtime_fingerprint,
                          std::uint64_t planning_fingerprint) noexcept;

[[nodiscard]] std::string formatTrajectoryDeliveryAtReceive(
    const TrajectoryDeliveryDiagnostics* delivery, std::uint64_t path_stamp_ns,
    std::int64_t receive_stamp_ns, Point2 actual_receive_position);

} // namespace drone_city_nav
