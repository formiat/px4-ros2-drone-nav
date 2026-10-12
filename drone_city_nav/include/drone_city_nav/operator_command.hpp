#pragma once

#include "drone_city_nav/types.hpp"

#include <cmath>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>

namespace drone_city_nav {

// The operator's commands to a flying vehicle (roadmap item 27 block 1,
// specification K31), on the one channel a goal enters the mission by:
// "stop" holds where the vehicle is, "land" lands it there, "home" gives the
// goal up for the start by the way it came (K20), "goal X Y Z" flies to a new
// goal in the map frame.
enum class OperatorCommandKind : std::uint8_t {
  kStop,
  kLand,
  kHome,
  kGoal
};

struct OperatorCommand {
  OperatorCommandKind kind{OperatorCommandKind::kStop};
  Point3 goal{};
};

[[nodiscard]] inline std::optional<OperatorCommand>
parseOperatorCommand(const std::string& text) {
  std::istringstream words{text};
  std::string verb;
  words >> verb;
  if (verb == "stop" || verb == "land" || verb == "home") {
    std::string rest;
    if (words >> rest) {
      return std::nullopt;
    }
    return OperatorCommand{.kind = verb == "stop"   ? OperatorCommandKind::kStop
                                   : verb == "land" ? OperatorCommandKind::kLand
                                                    : OperatorCommandKind::kHome};
  }
  if (verb == "goal") {
    Point3 goal{};
    std::string rest;
    if (!(words >> goal.x >> goal.y >> goal.z) || (words >> rest) ||
        !std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(goal.z)) {
      return std::nullopt;
    }
    return OperatorCommand{.kind = OperatorCommandKind::kGoal, .goal = goal};
  }
  return std::nullopt;
}

} // namespace drone_city_nav
