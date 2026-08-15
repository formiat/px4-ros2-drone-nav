#pragma once

#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/types.hpp"

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

namespace drone_city_nav::test {

enum class AdvancedPassageFixtureKind : std::uint8_t {
  kSlopedTunnel,
  kVerticalShaft,
  kArchTunnel,
  kCurvedTunnel,
  kTJunction,
  kXJunction,
  kWideHangar,
};

struct AdvancedPassageExpectation {
  bool should_extract_passage{true};
  std::size_t minimum_portal_count{2U};
  std::size_t minimum_segment_count{1U};
  std::vector<Point3> open_space_seeds;
  std::vector<std::vector<Point3>> raw_safe_reference_paths;
};

struct AdvancedPassageFixture {
  AdvancedPassageFixtureKind kind{AdvancedPassageFixtureKind::kSlopedTunnel};
  std::string_view name;
  OccupancyGrid3D occupancy;
  AdvancedPassageExpectation expectation;
};

[[nodiscard]] std::vector<AdvancedPassageFixtureKind> advancedPassageFixtureKinds();

[[nodiscard]] AdvancedPassageFixture
buildAdvancedPassageFixture(AdvancedPassageFixtureKind kind);

} // namespace drone_city_nav::test
