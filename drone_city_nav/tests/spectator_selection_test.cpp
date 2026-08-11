#include "drone_city_nav/spectator_selection.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace drone_city_nav {
namespace {

TEST(SpectatorSelectionTest, ParsesSupportedPolicies) {
  EXPECT_EQ(parseSpectatorReselectionPolicy("first_living"),
            SpectatorReselectionPolicy::kFirstLiving);
  EXPECT_EQ(parseSpectatorReselectionPolicy("next_living"),
            SpectatorReselectionPolicy::kNextLiving);
  EXPECT_FALSE(parseSpectatorReselectionPolicy("nearest").has_value());
}

TEST(SpectatorSelectionTest, FirstLivingReturnsToLowestLivingIndex) {
  SpectatorSelection selection{4U, 2U, SpectatorReselectionPolicy::kFirstLiving};

  const std::optional<std::size_t> replacement = selection.markDestroyed(2U);

  EXPECT_EQ(replacement, std::optional<std::size_t>{0U});
  EXPECT_EQ(selection.currentIndex(), 0U);
}

TEST(SpectatorSelectionTest, NextLivingAdvancesWithoutReturningToFirst) {
  SpectatorSelection selection{4U, 2U, SpectatorReselectionPolicy::kNextLiving};

  const std::optional<std::size_t> replacement = selection.markDestroyed(2U);

  EXPECT_EQ(replacement, std::optional<std::size_t>{3U});
  EXPECT_EQ(selection.currentIndex(), 3U);
}

TEST(SpectatorSelectionTest, NextLivingSkipsDestroyedAndWraps) {
  SpectatorSelection selection{4U, 2U, SpectatorReselectionPolicy::kNextLiving};
  EXPECT_FALSE(selection.markDestroyed(3U).has_value());

  const std::optional<std::size_t> replacement = selection.markDestroyed(2U);

  EXPECT_EQ(replacement, std::optional<std::size_t>{0U});
}

TEST(SpectatorSelectionTest, RetainsLastSelectionWhenNoVehicleLives) {
  SpectatorSelection selection{2U, 1U, SpectatorReselectionPolicy::kNextLiving};
  EXPECT_FALSE(selection.markDestroyed(0U).has_value());

  EXPECT_FALSE(selection.markDestroyed(1U).has_value());
  EXPECT_EQ(selection.currentIndex(), 1U);
  EXPECT_TRUE(selection.destroyed(1U));
}

TEST(SpectatorSelectionTest, RejectsInvalidInitialIndex) {
  EXPECT_THROW((SpectatorSelection{2U, 2U, SpectatorReselectionPolicy::kFirstLiving}),
               std::out_of_range);
}

} // namespace
} // namespace drone_city_nav
