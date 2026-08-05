#include "drone_city_nav/flight_envelope.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace drone_city_nav {
namespace {

TEST(FlightEnvelopeTest, IncludesMinimumAndExcludesMaximum) {
  const FlightEnvelopeConfig config{.minimum_target_z_m = 1.0,
                                    .maximum_target_z_m = 32.0};
  EXPECT_TRUE(insideFlightEnvelope(1.0, config));
  EXPECT_TRUE(insideFlightEnvelope(31.999, config));
  EXPECT_FALSE(insideFlightEnvelope(0.999, config));
  EXPECT_FALSE(insideFlightEnvelope(32.0, config));
}

TEST(FlightEnvelopeTest, RejectsNonFiniteAltitudeAndInvalidConfiguration) {
  EXPECT_EQ(evaluateFlightEnvelopeAltitude(std::numeric_limits<double>::quiet_NaN(),
                                           FlightEnvelopeConfig{}),
            FlightEnvelopeStatus::kNonFiniteAltitude);
  EXPECT_EQ(evaluateFlightEnvelopeAltitude(
                10.0, FlightEnvelopeConfig{.minimum_target_z_m = 10.0,
                                           .maximum_target_z_m = 10.0}),
            FlightEnvelopeStatus::kInvalidConfiguration);
}

TEST(FlightEnvelopeTest, SegmentRequiresBothEndpointsInside) {
  EXPECT_TRUE(
      segmentInsideFlightEnvelope(Point3{0.0, 0.0, 1.0}, Point3{1.0, 1.0, 31.0}, {}));
  EXPECT_FALSE(
      segmentInsideFlightEnvelope(Point3{0.0, 0.0, 1.0}, Point3{1.0, 1.0, 32.0}, {}));
}

} // namespace
} // namespace drone_city_nav
