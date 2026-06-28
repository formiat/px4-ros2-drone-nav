#include "drone_city_nav/offboard_blackbox.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <sstream>
#include <string>

namespace drone_city_nav {

TEST(OffboardBlackbox, WritesJsonPrimitives) {
  std::ostringstream stream;

  writeBlackboxJsonBool(stream, true);
  stream << ",";
  writeBlackboxJsonBool(stream, false);
  stream << ",";
  writeBlackboxJsonNumberOrNull(stream, 4.25);
  stream << ",";
  writeBlackboxJsonNumberOrNull(stream, std::numeric_limits<double>::quiet_NaN());
  stream << ",";
  writeBlackboxStringField(stream, "mode", "velocity_cruise");

  EXPECT_EQ(stream.str(), "true,false,4.25,null,\"mode\":\"velocity_cruise\"");
}

TEST(OffboardBlackbox, WritesPathIdContract) {
  std::ostringstream stream;

  writeBlackboxPathId(stream, OffboardBlackboxPathId{7U, 42U, true, 123456U});

  EXPECT_EQ(stream.str(), "\"path_id\":{\"local_update\":7,\"planner\":42,"
                          "\"planner_seen\":true,\"stamp_ns\":123456}");
}

} // namespace drone_city_nav
