#include "drone_city_nav/offboard_blackbox.hpp"

#include <cmath>
#include <ostream>

namespace drone_city_nav {

void writeBlackboxJsonBool(std::ostream& stream, const bool value) {
  stream << (value ? "true" : "false");
}

void writeBlackboxJsonNumberOrNull(std::ostream& stream, const double value) {
  if (std::isfinite(value)) {
    stream << value;
    return;
  }
  stream << "null";
}

void writeBlackboxPathId(std::ostream& stream, const OffboardBlackboxPathId& path_id) {
  stream << "\"path_id\":{\"local_update\":" << path_id.local_update
         << ",\"planner\":" << path_id.planner << ",\"planner_seen\":";
  writeBlackboxJsonBool(stream, path_id.planner_seen);
  stream << ",\"stamp_ns\":" << path_id.stamp_ns << "}";
}

void writeBlackboxStringField(std::ostream& stream, const std::string_view key,
                              const std::string_view value) {
  stream << "\"" << key << "\":\"" << value << "\"";
}

} // namespace drone_city_nav
