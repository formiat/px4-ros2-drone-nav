#include "drone_city_nav/spectator_diagnostics_selection.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(SpectatorDiagnosticsSelectionTest, UngatedSelectionRemainsActive) {
  SpectatorDiagnosticsSelection selection;

  EXPECT_FALSE(selection.gated());
  EXPECT_TRUE(selection.selected());
  EXPECT_FALSE(selection.select("civilian_2"));
  EXPECT_TRUE(selection.selected());
}

TEST(SpectatorDiagnosticsSelectionTest, GatedSelectionTracksVehicleId) {
  SpectatorDiagnosticsSelection selection{"civilian_1"};

  EXPECT_TRUE(selection.gated());
  EXPECT_FALSE(selection.selected());
  EXPECT_FALSE(selection.select("civilian_0"));
  EXPECT_TRUE(selection.select("civilian_1"));
  EXPECT_TRUE(selection.selected());
  EXPECT_FALSE(selection.select("civilian_1"));
  EXPECT_TRUE(selection.select("civilian_2"));
  EXPECT_FALSE(selection.selected());
}

} // namespace
} // namespace drone_city_nav
