#pragma once

#include <cstdint>
#include <iosfwd>
#include <string_view>

namespace drone_city_nav {

struct OffboardBlackboxPathId {
  std::uint64_t local_update{0U};
  std::uint64_t planner{0U};
  bool planner_seen{false};
  std::uint64_t stamp_ns{0U};
};

void writeBlackboxJsonBool(std::ostream& stream, bool value);

void writeBlackboxJsonNumberOrNull(std::ostream& stream, double value);

void writeBlackboxPathId(std::ostream& stream, const OffboardBlackboxPathId& path_id);

void writeBlackboxStringField(std::ostream& stream, std::string_view key,
                              std::string_view value);

} // namespace drone_city_nav
