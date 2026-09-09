#include "drone_city_nav/local_hold_pin.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {

TEST(LocalHoldPin, KeepsThePinWhileTheVehicleStillStandsOnIt) {
  LocalHoldPin pin;
  EXPECT_FALSE(pin.pin().has_value());

  EXPECT_TRUE(pin.acquire(Point3{1.0, 2.0, 3.0}));
  ASSERT_TRUE(pin.pin().has_value());
  EXPECT_DOUBLE_EQ(pin.pin()->x, 1.0);

  // Held again a few centimetres away, while the hold is in force: same pin.
  EXPECT_FALSE(pin.acquire(Point3{1.05, 2.0, 3.0}));
  EXPECT_DOUBLE_EQ(pin.pin()->x, 1.0);

  // A horizon took over and left the vehicle within the tolerance: same pin.
  pin.release();
  EXPECT_FALSE(pin.acquire(Point3{1.1, 2.1, 3.0}));
  EXPECT_DOUBLE_EQ(pin.pin()->x, 1.0);

  // A horizon carried the vehicle away: a new pin where it stands now.
  pin.release();
  EXPECT_TRUE(pin.acquire(Point3{4.0, 2.0, 3.0}));
  EXPECT_DOUBLE_EQ(pin.pin()->x, 4.0);

  pin.reset();
  EXPECT_FALSE(pin.pin().has_value());
}

} // namespace drone_city_nav
