#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>

namespace drone_city_nav {
namespace {

[[nodiscard]] SweptFootprintConfig pointFootprint() noexcept {
  return SweptFootprintConfig{.radius_m = 0.0,
                              .lower_extent_m = 0.0,
                              .upper_extent_m = 0.0,
                              .perimeter_samples = 0U,
                              .radial_rings = 0U,
                              .axial_samples = 0U,
                              .sweep_step_m = 0.25};
}

TEST(OccupiedCollisionOracle3DTest, TreatsUnknownAndKnownFreeAsEquallyClear) {
  ObservedOccupancyGrid3D observed{GridBounds3D{0.0, 0.0, 0.0, 1.0, 4, 4, 4}};
  ASSERT_TRUE(observed.setState(GridIndex3D{1, 1, 1}, ObservedVoxelState::kFree));
  const OccupiedCollisionOracle3D oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = &observed,
      .static_occupancy = nullptr,
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = pointFootprint(),
      .flight_envelope = FlightEnvelopeConfig{0.0, 10.0},
  }};

  EXPECT_TRUE(oracle.validatePoint(Point3{1.5, 1.5, 1.5}).clear());
  EXPECT_TRUE(oracle.validatePoint(Point3{2.5, 1.5, 1.5}).clear());
  EXPECT_TRUE(oracle.validatePoint(Point3{20.0, 20.0, 1.5}).clear());
}

TEST(OccupiedCollisionOracle3DTest, UnionsObservedAndStaticOccupiedEvidence) {
  ObservedOccupancyGrid3D observed{GridBounds3D{0.0, 0.0, 0.0, 1.0, 5, 3, 3}};
  ASSERT_TRUE(observed.setState(GridIndex3D{1, 1, 1}, ObservedVoxelState::kOccupied));
  OccupancyGrid3D known{GridBounds3D{0.0, 0.0, 0.0, 1.0, 5, 3, 3}};
  known.setOccupied(GridIndex3D{3, 1, 1});
  const OccupiedCollisionOracle3D oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = &observed,
      .static_occupancy = &known,
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = pointFootprint(),
      .flight_envelope = FlightEnvelopeConfig{0.0, 10.0},
  }};

  EXPECT_EQ(oracle.validatePoint(Point3{1.5, 1.5, 1.5}).status,
            OccupiedCollisionStatus3D::kRawCollision);
  EXPECT_EQ(oracle.validatePoint(Point3{3.5, 1.5, 1.5}).status,
            OccupiedCollisionStatus3D::kRawCollision);
  EXPECT_TRUE(oracle.validatePoint(Point3{2.5, 1.5, 1.5}).clear());
}

TEST(OccupiedCollisionOracle3DTest, FlightEnvelopeIsTheOnlyNonOccupancyHardGate) {
  const OccupiedCollisionOracle3D oracle{OccupiedCollisionWorld3D{
      .observed_occupancy = nullptr,
      .static_occupancy = nullptr,
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = pointFootprint(),
      .flight_envelope = FlightEnvelopeConfig{1.0, 4.0},
  }};

  EXPECT_EQ(
      oracle.validateSegment(Point3{0.0, 0.0, 1.0}, {}, Point3{10.0, 0.0, 3.999}, {})
          .status,
      OccupiedCollisionStatus3D::kClear);
  EXPECT_EQ(oracle.validatePoint(Point3{0.0, 0.0, 4.0}).status,
            OccupiedCollisionStatus3D::kOutsideFlightEnvelope);
  EXPECT_EQ(
      oracle.validatePoint(Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 2.0})
          .status,
      OccupiedCollisionStatus3D::kInvalidInput);
}

TEST(OccupiedCollisionOracle3DTest, RejectsMalformedHardCollisionInputs) {
  const std::array raw_points{
      Point3{std::numeric_limits<double>::quiet_NaN(), 0.0, 2.0}};
  const OccupiedCollisionOracle3D malformed_world{OccupiedCollisionWorld3D{
      .raw_point_cloud = raw_points,
      .footprint = pointFootprint(),
      .flight_envelope = FlightEnvelopeConfig{0.0, 10.0},
  }};
  const OccupiedCollisionOracle3D valid_world{OccupiedCollisionWorld3D{
      .observed_occupancy = nullptr,
      .static_occupancy = nullptr,
      .planar_occupancy = nullptr,
      .raw_point_cloud = {},
      .launch_support_contact = nullptr,
      .footprint = pointFootprint(),
      .flight_envelope = FlightEnvelopeConfig{0.0, 10.0},
  }};

  EXPECT_EQ(malformed_world.validatePoint(Point3{0.0, 0.0, 2.0}).status,
            OccupiedCollisionStatus3D::kInvalidInput);
  EXPECT_EQ(valid_world
                .validateSegment(Point3{0.0, 0.0, 2.0}, FootprintBodyAxis{},
                                 Point3{1.0, 0.0, 2.0},
                                 FootprintBodyAxis{0.0, 0.0, 0.0})
                .status,
            OccupiedCollisionStatus3D::kInvalidInput);
}

} // namespace
} // namespace drone_city_nav
