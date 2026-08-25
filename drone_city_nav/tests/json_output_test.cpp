#include "drone_city_nav/json_output.hpp"

#include <gtest/gtest.h>

#include <iomanip>
#include <limits>

namespace drone_city_nav {
namespace {

TEST(JsonOutputStreamTest, ReplacesEveryNonFiniteFloatingPointValueWithNull) {
  JsonOutputStream json;
  json << std::fixed << std::setprecision(2) << "{\"finite\":" << 1.25
       << ",\"nan\":" << std::numeric_limits<double>::quiet_NaN()
       << ",\"positive_inf\":" << std::numeric_limits<float>::infinity()
       << ",\"negative_inf\":" << -std::numeric_limits<long double>::infinity()
       << ",\"count\":" << 7U << '}';

  EXPECT_EQ(json.str(), "{\"finite\":1.25,\"nan\":null,\"positive_inf\":null,"
                        "\"negative_inf\":null,\"count\":7}");
}

} // namespace
} // namespace drone_city_nav
