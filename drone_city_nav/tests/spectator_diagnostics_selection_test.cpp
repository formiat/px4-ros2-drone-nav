#include "drone_city_nav/spectator_diagnostics_selection.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(SpectatorDiagnosticsSelectionTest, UngatedSelectionRemainsActive) {
  SpectatorDiagnosticsSelection selection;

  EXPECT_FALSE(selection.gated());
  EXPECT_TRUE(selection.selected());
  EXPECT_FALSE(selection.select("interceptor_2"));
  EXPECT_TRUE(selection.selected());
}

TEST(SpectatorDiagnosticsSelectionTest, GatedSelectionTracksVehicleId) {
  SpectatorDiagnosticsSelection selection{"interceptor_1"};

  EXPECT_TRUE(selection.gated());
  EXPECT_FALSE(selection.selected());
  EXPECT_FALSE(selection.select("interceptor_0"));
  EXPECT_TRUE(selection.select("interceptor_1"));
  EXPECT_TRUE(selection.selected());
  EXPECT_FALSE(selection.select("interceptor_1"));
  EXPECT_TRUE(selection.select("interceptor_2"));
  EXPECT_FALSE(selection.selected());
}

} // namespace
} // namespace drone_city_nav
