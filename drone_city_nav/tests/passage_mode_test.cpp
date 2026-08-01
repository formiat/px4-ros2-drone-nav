#include "drone_city_nav/passage_mode.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

TEST(PassageModeTest, EnablesConfiguredPassagesOnlyForStaticMode) {
  EXPECT_TRUE(semanticPassagesEnabled(true, true));
  EXPECT_FALSE(semanticPassagesEnabled(false, true));
  EXPECT_FALSE(semanticPassagesEnabled(true, false));
  EXPECT_FALSE(semanticPassagesEnabled(false, false));
}

} // namespace
} // namespace drone_city_nav
