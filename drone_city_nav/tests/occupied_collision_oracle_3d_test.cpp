#include "drone_city_nav/occupied_collision_oracle_3d.hpp"

#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <span>

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

TEST(OccupiedCollisionOracle3DTest, ProprioceptiveSeedMakesTheOwnPoseAValidDeparture) {
  // Observed evidence closed in on the vehicle: one occupied voxel overlaps the
  // body at its pose. Without the seed the pose itself is a collision and no
  // motion from it validates; with the seed, departing is clear and the only
  // rejected motion is a further approach.
  ObservedOccupancyGrid3D observed{GridBounds3D{0.0, 0.0, 0.0, 0.25, 40, 40, 40}};
  ASSERT_TRUE(
      observed.setState(GridIndex3D{24, 20, 20}, ObservedVoxelState::kOccupied));
  const SweptFootprintConfig footprint{.radius_m = 0.5,
                                       .lower_extent_m = 0.25,
                                       .upper_extent_m = 0.25,
                                       .sweep_step_m = 0.125};
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{5.6, 5.1, 5.1},
      .body_axis = FootprintBodyAxis{},
      .footprint = footprint,
      .contact_tolerance_m = 0.125,
  };
  const OccupiedCollisionOracle3D strict{OccupiedCollisionWorld3D{
      .observed_occupancy = &observed,
      .footprint = footprint,
      .flight_envelope = FlightEnvelopeConfig{0.0, 10.0},
  }};
  const OccupiedCollisionOracle3D seeded{OccupiedCollisionWorld3D{
      .observed_occupancy = &observed,
      .proprioceptive_free_space_seed = &seed,
      .footprint = footprint,
      .flight_envelope = FlightEnvelopeConfig{0.0, 10.0},
  }};
  const FootprintBodyAxis axis{};

  EXPECT_EQ(strict.validatePoint(seed.position, axis).status,
            OccupiedCollisionStatus3D::kRawCollision);
  EXPECT_TRUE(seeded.validatePoint(seed.position, axis).clear());
  EXPECT_EQ(
      strict.validateSegment(seed.position, axis, Point3{5.0, 5.1, 5.1}, axis).status,
      OccupiedCollisionStatus3D::kRawCollision);
  EXPECT_TRUE(
      seeded.validateSegment(seed.position, axis, Point3{5.0, 5.1, 5.1}, axis).clear());
  // The exemption places no condition on the direction of motion: free space
  // stays traversable in every direction from a body already in contact. The
  // body itself overlaps this voxel at the seed, so it is contact through and
  // through, not margin evidence the body may never reach.
  EXPECT_TRUE(
      seeded.validateSegment(seed.position, axis, Point3{5.9, 5.1, 5.1}, axis).clear());
}

TEST(OccupiedCollisionOracle3DTest, ContactExemptsTheEnvelopeNeverTheBody) {
  // The vehicle rests beside a wall: the 0.82 m envelope overlaps the wall's
  // voxels and the lidar returns on its face, the 0.55 m body does not. The
  // exemption lets it hold and fly away from the face; the pose whose body
  // would reach the face is a collision like any other.
  ObservedOccupancyGrid3D observed{GridBounds3D{0.0, 0.0, 0.0, 0.25, 40, 40, 40}};
  for (int x = 16; x < 24; ++x) {
    for (int z = 18; z < 23; ++z) {
      ASSERT_TRUE(
          observed.setState(GridIndex3D{x, 24, z}, ObservedVoxelState::kOccupied));
    }
  }
  const std::array<Point3, 3> face{Point3{4.6, 6.0, 5.1}, Point3{5.0, 6.0, 5.1},
                                   Point3{5.4, 6.0, 5.1}};
  const SweptFootprintConfig footprint{.radius_m = 0.82,
                                       .lower_extent_m = 0.25,
                                       .upper_extent_m = 0.25,
                                       .body_radius_m = 0.55,
                                       .sweep_step_m = 0.125};
  // Centre 0.75 m from the face: inside the envelope, outside the body.
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{5.0, 5.25, 5.1},
      .body_axis = FootprintBodyAxis{},
      .footprint = footprint,
      .contact_tolerance_m = 0.125,
  };
  const FootprintBodyAxis axis{};
  const OccupiedCollisionOracle3D strict{OccupiedCollisionWorld3D{
      .observed_occupancy = &observed,
      .raw_point_cloud = std::span<const Point3>{face},
      .footprint = footprint,
      .flight_envelope = FlightEnvelopeConfig{0.0, 10.0},
  }};
  const OccupiedCollisionOracle3D seeded{OccupiedCollisionWorld3D{
      .observed_occupancy = &observed,
      .raw_point_cloud = std::span<const Point3>{face},
      .proprioceptive_free_space_seed = &seed,
      .footprint = footprint,
      .flight_envelope = FlightEnvelopeConfig{0.0, 10.0},
  }};

  EXPECT_EQ(strict.validatePoint(seed.position, axis).status,
            OccupiedCollisionStatus3D::kRawCollision);
  EXPECT_TRUE(seeded.validatePoint(seed.position, axis).clear());
  // Holding and flying away from the face: clear.
  EXPECT_TRUE(seeded.validateSegment(seed.position, axis, seed.position, axis).clear());
  EXPECT_TRUE(
      seeded.validateSegment(seed.position, axis, Point3{5.0, 4.6, 5.1}, axis).clear());
  // A drift of 0.3 m into the face brings the body onto the wall: a collision,
  // by the lidar returns and by the voxels alike.
  EXPECT_EQ(
      seeded.validateSegment(seed.position, axis, Point3{5.0, 5.55, 5.1}, axis).status,
      OccupiedCollisionStatus3D::kRawCollision);
  const OccupiedCollisionOracle3D voxels_only{OccupiedCollisionWorld3D{
      .observed_occupancy = &observed,
      .proprioceptive_free_space_seed = &seed,
      .footprint = footprint,
      .flight_envelope = FlightEnvelopeConfig{0.0, 10.0},
  }};
  EXPECT_TRUE(voxels_only.validatePoint(seed.position, axis).clear());
  EXPECT_EQ(
      voxels_only.validateSegment(seed.position, axis, Point3{5.0, 5.55, 5.1}, axis)
          .status,
      OccupiedCollisionStatus3D::kRawCollision);
}

TEST(OccupiedCollisionOracle3DTest, ContactEvidenceIsTiedToTheSeedNotTheWorld) {
  // Two seeds on the same world: the vehicle standing in the contact voxel
  // flies freely around it, while a vehicle two metres away sees the same
  // voxel as an obstacle. Contact is a property of where the body stands.
  ObservedOccupancyGrid3D observed{GridBounds3D{0.0, 0.0, 0.0, 0.25, 40, 40, 40}};
  ASSERT_TRUE(
      observed.setState(GridIndex3D{24, 20, 20}, ObservedVoxelState::kOccupied));
  const SweptFootprintConfig footprint{.radius_m = 0.5,
                                       .lower_extent_m = 0.25,
                                       .upper_extent_m = 0.25,
                                       .sweep_step_m = 0.125};
  const FootprintBodyAxis axis{};
  const Point3 contact_position{5.6, 5.1, 5.1};
  const Point3 distant_position{3.6, 5.1, 5.1};
  const ProprioceptiveFreeSpaceSeed3D contact_seed{
      .position = contact_position,
      .body_axis = axis,
      .footprint = footprint,
      .contact_tolerance_m = 0.125,
  };
  const ProprioceptiveFreeSpaceSeed3D distant_seed{
      .position = distant_position,
      .body_axis = axis,
      .footprint = footprint,
      .contact_tolerance_m = 0.125,
  };
  const auto oracle_for = [&](const ProprioceptiveFreeSpaceSeed3D& seed) {
    return OccupiedCollisionOracle3D{OccupiedCollisionWorld3D{
        .observed_occupancy = &observed,
        .proprioceptive_free_space_seed = &seed,
        .footprint = footprint,
        .flight_envelope = FlightEnvelopeConfig{0.0, 10.0},
    }};
  };

  EXPECT_TRUE(oracle_for(contact_seed).validatePoint(contact_position, axis).clear());
  EXPECT_TRUE(oracle_for(contact_seed)
                  .validateSegment(contact_position, axis, distant_position, axis)
                  .clear());
  EXPECT_EQ(oracle_for(distant_seed)
                .validateSegment(distant_position, axis, contact_position, axis)
                .status,
            OccupiedCollisionStatus3D::kRawCollision);
}

} // namespace
} // namespace drone_city_nav
