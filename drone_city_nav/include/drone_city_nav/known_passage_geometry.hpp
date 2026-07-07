#pragma once

#include "drone_city_nav/known_passage_map.hpp"

#include <optional>

namespace drone_city_nav {

struct KnownPassageOpeningFrame {
  Point2 center{};
  Point2 normal{1.0, 0.0};
  Point2 lateral{0.0, 1.0};
  double half_width_m{0.0};
  double half_depth_m{0.0};
};

struct KnownPassageOpeningWorldPoint {
  Point2 point{};
  double z_m{0.0};
  double s_m{0.0};
};

struct KnownPassageOpeningLocalPoint {
  double u_m{0.0};
  double v_m{0.0};
  double z_m{0.0};
  double s_m{0.0};
};

[[nodiscard]] std::optional<KnownPassageOpeningFrame>
knownPassageOpeningFrame(const PassageOpening& opening);

[[nodiscard]] KnownPassageOpeningLocalPoint
knownPassageOpeningLocalPoint(const KnownPassageOpeningWorldPoint& point,
                              const KnownPassageOpeningFrame& frame);

[[nodiscard]] double
knownPassageOpeningLateralClearanceM(const KnownPassageOpeningLocalPoint& point,
                                     const KnownPassageOpeningFrame& frame) noexcept;

[[nodiscard]] double
knownPassageOpeningVerticalClearanceM(const KnownPassageOpeningLocalPoint& point,
                                      const PassageOpening& opening) noexcept;

[[nodiscard]] double
knownPassageOpeningPassageClearanceM(const KnownPassageOpeningLocalPoint& point,
                                     const PassageOpening& opening,
                                     const KnownPassageOpeningFrame& frame) noexcept;

[[nodiscard]] double
knownPassageOpeningSignedVolumeMarginM(const KnownPassageOpeningLocalPoint& point,
                                       const PassageOpening& opening,
                                       const KnownPassageOpeningFrame& frame) noexcept;

[[nodiscard]] Point2
knownPassageOpeningGateEntryPoint(const PassageOpening& opening,
                                  const KnownPassageOpeningFrame& frame) noexcept;

[[nodiscard]] Point2
knownPassageOpeningGateExitPoint(const PassageOpening& opening,
                                 const KnownPassageOpeningFrame& frame) noexcept;

} // namespace drone_city_nav
