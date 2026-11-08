#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <numbers>

#include "survey_mission.hpp"

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecond{1'000'000'000};
constexpr double kResolution{0.25};

// A 16 x 12 x 6 m box of memory. A room of 8 x 8 x 4 m observed whole, its
// walls occupied, the air outside it unknown; with a door, a gap of 1.5 x 2 m
// in the east wall whose cells are observed free, and 6 m of unknown beyond.
[[nodiscard]] ObservedOccupancyGrid3D room(const bool door) {
  ObservedOccupancyGrid3D grid{GridBounds3D{.origin_x = 0.0,
                                            .origin_y = 0.0,
                                            .origin_z = 0.0,
                                            .resolution_m = kResolution,
                                            .width_cells = 64,
                                            .height_cells = 48,
                                            .depth_cells = 24}};
  for (int z = 4; z < 20; ++z) {
    for (int y = 8; y < 40; ++y) {
      for (int x = 8; x < 40; ++x) {
        const bool wall = x == 8 || x == 39 || y == 8 || y == 39 || z == 4 || z == 19;
        const bool opening = door && x == 39 && y >= 20 && y < 26 && z >= 5 && z < 13;
        grid.setState(GridIndex3D{x, y, z}, wall && !opening
                                                ? ObservedVoxelState::kOccupied
                                                : ObservedVoxelState::kFree);
      }
    }
  }
  return grid;
}

[[nodiscard]] SurveyConfig config() {
  SurveyConfig config;
  config.volume_minimum = Point3{0.0, 0.0, 0.0};
  config.volume_maximum = Point3{16.0, 12.0, 6.0};
  config.envelope =
      FlightEnvelopeConfig{.minimum_target_z_m = 0.5, .maximum_target_z_m = 5.5};
  return config;
}

const Point3 kVehicle{4.0, 6.0, 2.5};

TEST(SurveyMissionTest, AClosedRoomHoldsNoFrontierAndTheSurveyGoesHome) {
  const ObservedOccupancyGrid3D grid = room(false);
  const SurveyReading reading = readSurvey(grid, kVehicle, config(), {}, {});
  EXPECT_EQ(reading.frontier_cells, 0U);
  EXPECT_TRUE(reading.vehicle_placed);
  EXPECT_FALSE(reading.budget_exhausted);
  EXPECT_TRUE(reading.complete());
  SurveyMission mission{config()};
  mission.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  mission.recordPose(kVehicle, 0.0, true, kSecond);
  mission.applyReading(reading);
  const SurveyStep step = mission.step(2 * kSecond);
  EXPECT_EQ(step.command, SurveyCommand::kHome);
  ASSERT_EQ(step.events.size(), 1U);
  EXPECT_EQ(step.events.front().rfind("SURVEY_DONE", 0), 0U);
  EXPECT_TRUE(mission.done());
  EXPECT_EQ(mission.step(3 * kSecond).command, SurveyCommand::kNone);
}

TEST(SurveyMissionTest, ADoorIsOneFrontierLookedAtFromInsideTheRoom) {
  const ObservedOccupancyGrid3D grid = room(true);
  const SurveyReading reading = readSurvey(grid, kVehicle, config(), {}, {});
  // The door's cells whose unknown beyond holds the body (K16's rule): not
  // those beside the jambs, the sill or the lintel.
  EXPECT_GE(reading.frontier_cells, 8U);
  EXPECT_LT(reading.frontier_cells, 6U * 8U);
  ASSERT_EQ(reading.frontiers.size(), 1U);
  const SurveyFrontier& door = reading.frontiers.front();
  EXPECT_EQ(door.cells, reading.frontier_cells);
  EXPECT_NEAR(door.centroid.x, 9.875, 1.0e-9);
  // The vantage stands the standoff short of the door on the vehicle's side, inside the
  // room.
  EXPECT_LT(door.vantage.x, door.centroid.x);
  EXPECT_GT(door.vantage.x, door.centroid.x - 3.5);
  EXPECT_GT(door.route_m, 0.0);
  // Beyond the door nothing was observed: no far vantage to pass through.
  EXPECT_FALSE(door.far_vantage_known);
  SurveyMission mission{config()};
  mission.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  mission.recordPose(kVehicle, 0.0, true, kSecond);
  mission.applyReading(reading);
  const SurveyStep chosen = mission.step(2 * kSecond);
  EXPECT_EQ(chosen.command, SurveyCommand::kGoal);
  EXPECT_NEAR(chosen.goal.x, door.vantage.x, 1.0e-9);
  ASSERT_EQ(chosen.events.size(), 1U);
  EXPECT_EQ(chosen.events.front().rfind("SURVEY_FRONTIER chosen", 0), 0U);
  // At the vantage facing the door for the dwell: looked at.
  mission.recordPose(door.vantage, 0.0, true, 10 * kSecond);
  mission.recordPose(door.vantage, 0.0, true, 12 * kSecond);
  const SurveyStep looked = mission.step(12 * kSecond);
  ASSERT_GE(looked.events.size(), 1U);
  EXPECT_EQ(looked.events.front().rfind("SURVEY_FRONTIER looked_at", 0), 0U);
  EXPECT_EQ(mission.looked().size(), door.cells);
  // The next reading leaves the looked-at cells out: nothing is left, home.
  const SurveyReading next =
      readSurvey(grid, door.vantage, config(), mission.looked(), {});
  EXPECT_EQ(next.frontier_cells, 0U);
  mission.applyReading(next);
  EXPECT_EQ(mission.step(13 * kSecond).command, SurveyCommand::kHome);
}

TEST(SurveyMissionTest, FacingAwayAtTheVantageWithNothingSeenBeyondTheWaitIsTheLook) {
  const ObservedOccupancyGrid3D grid = room(true);
  const SurveyReading reading = readSurvey(grid, kVehicle, config(), {}, {});
  ASSERT_EQ(reading.frontiers.size(), 1U);
  const SurveyFrontier door = reading.frontiers.front();
  // Beyond the door nothing was observed: no far vantage to pass through.
  EXPECT_FALSE(door.far_vantage_known);
  SurveyMission mission{config()};
  mission.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  mission.recordPose(kVehicle, 0.0, true, kSecond);
  mission.applyReading(reading);
  EXPECT_EQ(mission.step(2 * kSecond).command, SurveyCommand::kGoal);
  // At the vantage with the back to the door.
  for (int i = 0; i < 8; ++i) {
    mission.recordPose(door.vantage, std::numbers::pi, true, (10 + i) * kSecond);
    const SurveyStep step = mission.step((10 + i) * kSecond);
    if (!step.events.empty()) {
      // Settled; the stale reading still lists the door, so a goal follows.
      EXPECT_EQ(step.events.front().rfind("SURVEY_FRONTIER looked_at_from_vantage", 0),
                0U);
      EXPECT_EQ(mission.looked().size(), door.cells);
      return;
    }
  }
  FAIL() << "the wait at the vantage settled nothing";
}

// A corridor along x observed whole, its walls occupied, with a gap of the
// north wall unobserved: the corridor's cells beside the gap are the frontier,
// and the corridor beyond them is observed space a pass can go through.
[[nodiscard]] ObservedOccupancyGrid3D corridorWithGap() {
  ObservedOccupancyGrid3D grid{GridBounds3D{.origin_x = 0.0,
                                            .origin_y = 0.0,
                                            .origin_z = 0.0,
                                            .resolution_m = kResolution,
                                            .width_cells = 64,
                                            .height_cells = 48,
                                            .depth_cells = 24}};
  for (int z = 4; z < 21; ++z) {
    for (int y = 27; y < 46; ++y) {
      for (int x = 7; x < 58; ++x) {
        const bool wall = x == 7 || x == 57 || y == 27 || y == 45 || z == 4 || z == 20;
        const bool gap = y == 45 && x >= 30 && x < 37 && z >= 5 && z < 13;
        if (gap) {
          continue;
        }
        grid.setState(GridIndex3D{x, y, z},
                      wall ? ObservedVoxelState::kOccupied : ObservedVoxelState::kFree);
      }
    }
  }
  return grid;
}

TEST(SurveyMissionTest, APassGoesThroughObservedSpaceBeyondTheCentroid) {
  const ObservedOccupancyGrid3D grid = corridorWithGap();
  const Point3 vehicle{3.0, 9.0, 2.5};
  const SurveyReading reading = readSurvey(grid, vehicle, config(), {}, {});
  ASSERT_EQ(reading.frontiers.size(), 1U);
  const SurveyFrontier gap = reading.frontiers.front();
  EXPECT_GE(gap.cells, 8U);
  EXPECT_TRUE(gap.far_vantage_known);
  EXPECT_GT(gap.far_vantage.x, gap.centroid.x);
  EXPECT_LT(gap.vantage.x, gap.centroid.x);
  SurveyMission mission{config()};
  mission.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  mission.recordPose(vehicle, 0.0, true, kSecond);
  mission.applyReading(reading);
  EXPECT_EQ(mission.step(2 * kSecond).command, SurveyCommand::kGoal);
  // At the vantage facing back down the corridor, away from the gap.
  for (int i = 0; i < 8; ++i) {
    mission.recordPose(gap.vantage, std::numbers::pi, true, (10 + i) * kSecond);
    const SurveyStep step = mission.step((10 + i) * kSecond);
    if (step.command == SurveyCommand::kGoal) {
      EXPECT_NEAR(step.goal.x, gap.far_vantage.x, 1.0e-9);
      EXPECT_EQ(step.events.front().rfind("SURVEY_FRONTIER pass", 0), 0U);
      mission.recordPose(gap.far_vantage, std::numbers::pi, true, 30 * kSecond);
      const SurveyStep passed = mission.step(30 * kSecond);
      ASSERT_GE(passed.events.size(), 1U);
      EXPECT_EQ(passed.events.front().rfind("SURVEY_FRONTIER looked_at_on_pass", 0),
                0U);
      return;
    }
  }
  FAIL() << "no pass was commanded";
}

TEST(SurveyMissionTest, APlannerWithNoRouteToTheVantageForLongSettlesTheFrontier) {
  const ObservedOccupancyGrid3D grid = room(true);
  const SurveyReading reading = readSurvey(grid, kVehicle, config(), {}, {});
  ASSERT_EQ(reading.frontiers.size(), 1U);
  SurveyMission mission{config()};
  mission.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  mission.recordPose(kVehicle, std::numbers::pi, true, kSecond);
  mission.applyReading(reading);
  EXPECT_EQ(mission.step(2 * kSecond).command, SurveyCommand::kGoal);
  mission.recordRoute(false, 0.0, 3 * kSecond);
  mission.recordRoute(true, 5.0, 10 * kSecond); // a route once: the clock starts again
  mission.recordRoute(false, 0.0, 11 * kSecond);
  mission.recordRoute(false, 0.0, 40 * kSecond);
  EXPECT_TRUE(mission.step(40 * kSecond).events.empty());
  mission.recordRoute(false, 0.0, 42 * kSecond);
  const SurveyStep settled = mission.step(42 * kSecond);
  ASSERT_GE(settled.events.size(), 1U);
  EXPECT_EQ(settled.events.front().rfind("SURVEY_FRONTIER unroutable", 0), 0U);
  EXPECT_EQ(mission.looked().size(), reading.frontiers.front().cells);
}

TEST(SurveyMissionTest,
     TheBodyPassingThroughAFrontierCancelsItAndSmallClustersAreNotChosen) {
  ObservedOccupancyGrid3D grid = room(true);
  SurveyConfig small = config();
  small.cluster_minimum_cells = 64U;
  const SurveyReading unchosen = readSurvey(grid, kVehicle, small, {}, {});
  EXPECT_GE(unchosen.frontier_cells, 8U);
  EXPECT_TRUE(unchosen.frontiers.empty());
  EXPECT_EQ(unchosen.clusters, 0U);
  // The body passed through the door: the unknown cells beyond it count as known.
  SurveyMission mission{config()};
  mission.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  for (int i = 0; i <= 8; ++i) {
    mission.recordPose(Point3{9.5 + 0.25 * i, 5.75, 2.25}, 0.0, true,
                       (1 + i) * kSecond);
  }
  EXPECT_GT(mission.passed().size(), 0U);
  const SurveyReading passed =
      readSurvey(grid, kVehicle, config(), {}, mission.passed());
  EXPECT_LT(passed.frontier_cells, unchosen.frontier_cells);
}

// A 16 x 12 x 14 m box: the room of 8 x 8 x 4 m with its door high up (9 to
// 13 m), and a second room below it observed through nothing (0.25 to 0.75
// m), 8 m of unobserved floors between: its free cells next to unknown make a
// frontier whose only way from the vehicle runs through unobserved cells, and
// no observed cell lies within a vantage's reach of it (r2802).
[[nodiscard]] ObservedOccupancyGrid3D roomsApart() {
  ObservedOccupancyGrid3D grid{GridBounds3D{.origin_x = 0.0,
                                            .origin_y = 0.0,
                                            .origin_z = 0.0,
                                            .resolution_m = kResolution,
                                            .width_cells = 64,
                                            .height_cells = 48,
                                            .depth_cells = 56}};
  for (int z = 36; z < 52; ++z) {
    for (int y = 8; y < 40; ++y) {
      for (int x = 8; x < 40; ++x) {
        const bool wall = x == 8 || x == 39 || y == 8 || y == 39 || z == 36 || z == 51;
        const bool opening = x == 39 && y >= 20 && y < 26 && z >= 37 && z < 45;
        grid.setState(GridIndex3D{x, y, z}, wall && !opening
                                                ? ObservedVoxelState::kOccupied
                                                : ObservedVoxelState::kFree);
      }
    }
  }
  for (int z = 1; z < 3; ++z) {
    for (int y = 12; y < 36; ++y) {
      for (int x = 12; x < 36; ++x) {
        grid.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree);
      }
    }
  }
  return grid;
}

[[nodiscard]] SurveyConfig apartConfig() {
  SurveyConfig apart = config();
  apart.volume_maximum = Point3{16.0, 12.0, 14.0};
  apart.envelope =
      FlightEnvelopeConfig{.minimum_target_z_m = 0.25, .maximum_target_z_m = 13.5};
  return apart;
}

const Point3 kVehicleHigh{4.0, 6.0, 10.5};

TEST(SurveyMissionTest, AFrontierWithNoOpenVantageInSeenSpaceWaits) {
  // The room below is a frontier (its unknown holds the body), but no cell a
  // metre clear of anything unseen lies within reach of it, nor a way to it
  // through what was seen: it has no vantage, and the door is the one choice.
  const ObservedOccupancyGrid3D grid = roomsApart();
  const SurveyReading reading = readSurvey(grid, kVehicleHigh, apartConfig(), {}, {});
  EXPECT_EQ(reading.clusters, 2U);
  EXPECT_EQ(reading.clusters_without_vantage, 1U);
  ASSERT_EQ(reading.frontiers.size(), 1U);
  EXPECT_NEAR(reading.frontiers.front().centroid.x, 9.875, 1.0e-9); // the door
}

// The room with its east door and a second door in its north wall.
[[nodiscard]] ObservedOccupancyGrid3D roomTwoDoors() {
  ObservedOccupancyGrid3D grid = room(true);
  for (int z = 5; z < 13; ++z) {
    for (int x = 20; x < 26; ++x) {
      grid.setState(GridIndex3D{x, 39, z}, ObservedVoxelState::kFree);
    }
  }
  return grid;
}

TEST(SurveyMissionTest, APlannerRouteFarLongerThanTheReadingsDefersTheFrontier) {
  // One door chosen, the planner's way to it far longer than the reading's,
  // and the other door, nearer than that way: the first is deferred and
  // stays a frontier.
  const ObservedOccupancyGrid3D grid = roomTwoDoors();
  const SurveyReading reading = readSurvey(grid, kVehicle, config(), {}, {});
  ASSERT_EQ(reading.frontiers.size(), 2U);
  SurveyMission mission{config()};
  mission.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  mission.recordPose(kVehicle, std::numbers::pi, true, kSecond);
  mission.applyReading(reading);
  EXPECT_EQ(mission.step(2 * kSecond).command, SurveyCommand::kGoal);
  const Point3 first = mission.target()->centroid;
  mission.recordRoute(true, 120.0, 3 * kSecond);
  const SurveyStep deferred = mission.step(3 * kSecond);
  EXPECT_EQ(deferred.command, SurveyCommand::kGoal);
  ASSERT_GE(deferred.events.size(), 1U);
  EXPECT_EQ(deferred.events.front().rfind("SURVEY_FRONTIER deferred", 0), 0U);
  EXPECT_GT(distance3D(mission.target()->centroid, first), 1.0);
  EXPECT_TRUE(mission.looked().empty());
}

TEST(SurveyMissionTest, APinchedReadingIsNoVerdictAndTheVehicleRises) {
  const ObservedOccupancyGrid3D grid = room(true);
  SurveyReading wide = readSurvey(grid, kVehicle, config(), {}, {});
  ASSERT_GT(wide.reachable_voxels, 1000U);
  SurveyMission mission{config()};
  mission.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  mission.recordPose(kVehicle, 0.0, true, kSecond);
  mission.applyReading(wide);
  EXPECT_EQ(mission.step(2 * kSecond).command, SurveyCommand::kGoal);
  // A reading from a pinch: a sliver of the component, no frontier with a vantage.
  SurveyReading pinched = wide;
  pinched.reachable_voxels = 5U;
  pinched.frontiers.clear();
  pinched.reachable_members.clear();
  mission.applyReading(pinched);
  const SurveyStep rise = mission.step(3 * kSecond);
  EXPECT_EQ(rise.command, SurveyCommand::kGoal);
  EXPECT_NEAR(rise.goal.z, kVehicle.z + kRiseM, 1.0e-9);
  ASSERT_EQ(rise.events.size(), 1U);
  EXPECT_EQ(rise.events.front().rfind("SURVEY_RISE", 0), 0U);
  EXPECT_FALSE(mission.done());
  // Not repeated before its time; the wide reading again and the survey goes on.
  EXPECT_EQ(mission.step(4 * kSecond).command, SurveyCommand::kNone);
  mission.applyReading(wide);
  EXPECT_TRUE(mission.step(5 * kSecond).events.empty());
  EXPECT_FALSE(mission.done());
}

TEST(SurveyMissionTest, FrontierCellsTheFieldHeldCloseForTheDwellAreLookedAt) {
  // The door's cells, too few to be a cluster, in the field from 2 m for a
  // second: looked at cell by cell, and the survey ends.
  const ObservedOccupancyGrid3D grid = room(true);
  SurveyConfig large = config();
  large.cluster_minimum_cells = 64U;
  const SurveyReading reading = readSurvey(grid, kVehicle, large, {}, {});
  ASSERT_GE(reading.frontier_cells, 8U);
  EXPECT_TRUE(reading.frontiers.empty());
  SurveyMission mission{large};
  mission.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  const Point3 near{7.5, 5.75, 2.25};
  mission.recordPose(near, 0.0, true, kSecond);
  mission.applyReading(reading);
  // Complete and without a vantage: the end, unless the cells were looked at first.
  static_cast<void>(mission.step(kSecond));
  EXPECT_EQ(mission.looked().size(), 0U);
  SurveyMission held{large};
  held.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  held.recordPose(near, 0.0, true, kSecond);
  SurveyReading partial = reading;
  partial.budget_exhausted = true; // no verdict yet: the look alone is measured
  held.applyReading(partial);
  static_cast<void>(held.step(kSecond));
  EXPECT_EQ(held.looked().size(), 0U);
  held.recordPose(near, 0.0, true, 2 * kSecond + kSecond / 10);
  static_cast<void>(held.step(2 * kSecond + kSecond / 10));
  EXPECT_EQ(held.looked().size(), reading.frontier_cells);
  // Facing away, nothing is looked at.
  SurveyMission away{large};
  away.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  away.recordPose(near, std::numbers::pi, true, kSecond);
  away.applyReading(partial);
  static_cast<void>(away.step(kSecond));
  away.recordPose(near, std::numbers::pi, true, 3 * kSecond);
  static_cast<void>(away.step(3 * kSecond));
  EXPECT_EQ(away.looked().size(), 0U);
}

TEST(SurveyMissionTest, FrontierCellsWithinALooksFootprintAreTheSamePlace) {
  // One door cell looked at: the door's other cells, within a metre of it,
  // are the same place; a frontier a room away is not.
  const ObservedOccupancyGrid3D grid = roomTwoDoors();
  const SurveyReading before = readSurvey(grid, kVehicle, config(), {}, {});
  ASSERT_EQ(before.frontiers.size(), 2U);
  const SurveyFrontier& first = before.frontiers.front();
  // The door's cell nearest its middle.
  GridIndex3D middle = first.members.front();
  for (const GridIndex3D cell : first.members) {
    if (squaredDistance(grid.cellCenter(cell), first.centroid) <
        squaredDistance(grid.cellCenter(middle), first.centroid)) {
      middle = cell;
    }
  }
  const SurveyReading after =
      readSurvey(grid, kVehicle, config(), SurveyCellSet{middle}, {});
  ASSERT_EQ(after.frontiers.size(), 1U);
  EXPECT_GT(distance3D(after.frontiers.front().centroid, first.centroid), 3.0);
  EXPECT_EQ(after.frontier_cells, before.frontier_cells - first.cells);
}

TEST(SurveyMissionTest, ACorridorTwoMetresWideHoldsAVantageHighOverItsFloor) {
  // A corridor 2 m wide and 3 m tall observed for 8 m, unknown beyond: its
  // end is a frontier, and the vantage stands in it, off the walls and
  // 1.5 m over the floor (r2810: a ball of 1.5 m held in no corridor).
  ObservedOccupancyGrid3D grid{GridBounds3D{.origin_x = 0.0,
                                            .origin_y = 0.0,
                                            .origin_z = 0.0,
                                            .resolution_m = kResolution,
                                            .width_cells = 64,
                                            .height_cells = 48,
                                            .depth_cells = 24}};
  for (int z = 4; z < 18; ++z) {
    for (int y = 29; y < 39; ++y) {
      for (int x = 8; x < 41; ++x) {
        const bool wall = x == 8 || y == 29 || y == 38 || z == 4 || z == 17;
        grid.setState(GridIndex3D{x, y, z},
                      wall ? ObservedVoxelState::kOccupied : ObservedVoxelState::kFree);
      }
    }
  }
  const SurveyReading reading =
      readSurvey(grid, Point3{3.5, 8.375, 3.0}, config(), {}, {});
  ASSERT_EQ(reading.frontiers.size(), 1U);
  const SurveyFrontier& end = reading.frontiers.front();
  EXPECT_NEAR(end.centroid.x, 10.125, 1.0e-9);
  EXPECT_LT(end.vantage.x, end.centroid.x);
  EXPECT_NEAR(end.vantage.y, 8.375, 0.25);
  // The floor's top at 1.25 m: the vantage 1.5 m over it at least.
  EXPECT_GE(end.vantage.z, 1.25 + 1.5);
}

TEST(SurveyMissionTest, AFrontierBehindAWallOrUnknownIsNotLookedAt) {
  // The door's cells 3 m from a vehicle outside the room, facing them: the
  // line to them crosses unknown air and the wall, and the sensor did not
  // see them (r2810: the lidar's sphere looked through floors).
  const ObservedOccupancyGrid3D grid = room(true);
  SurveyConfig large = config();
  large.cluster_minimum_cells = 64U;
  SurveyReading reading = readSurvey(grid, kVehicle, large, {}, {});
  ASSERT_GE(reading.frontier_cells, 8U);
  reading.budget_exhausted = true; // no verdict: the look alone is measured
  SurveyMission mission{large};
  mission.setMemory(std::make_shared<const ObservedOccupancyGrid3D>(grid));
  const Point3 outside{11.0, 3.0, 2.25};
  const double facing = std::atan2(5.75 - outside.y, 9.875 - outside.x);
  mission.recordPose(outside, facing, true, kSecond);
  mission.applyReading(reading);
  static_cast<void>(mission.step(kSecond));
  mission.recordPose(outside, facing, true, 3 * kSecond);
  static_cast<void>(mission.step(3 * kSecond));
  EXPECT_EQ(mission.looked().size(), 0U);
  EXPECT_FALSE(lineOfSight(grid, outside, Point3{9.875, 5.75, 2.25}));
  EXPECT_TRUE(lineOfSight(grid, Point3{7.5, 5.75, 2.25}, Point3{9.875, 5.75, 2.25}));
}

TEST(SurveyMissionTest, AFrontierUnderAFloorHasNoVantageAboveIt) {
  // Two levels under one roof, a solid slab between them; the vehicle above,
  // an unobserved gap in the lower level's wall below it. Open cells above the
  // slab lie within reach of the gap's frontier, but the slab hides it: no
  // vantage (r2818: a vantage over a frontier under the floor, the vehicle
  // held over the stairwell until it sat on the floor).
  ObservedOccupancyGrid3D grid{GridBounds3D{.origin_x = 0.0,
                                            .origin_y = 0.0,
                                            .origin_z = 0.0,
                                            .resolution_m = kResolution,
                                            .width_cells = 64,
                                            .height_cells = 48,
                                            .depth_cells = 40}};
  for (int z = 4; z < 37; ++z) {
    for (int y = 8; y < 40; ++y) {
      for (int x = 8; x < 56; ++x) {
        const bool wall =
            x == 8 || x == 55 || y == 8 || y == 39 || z == 4 || z == 20 || z == 36;
        const bool gap = x == 8 && y >= 18 && y < 30 && z >= 7 && z < 17;
        if (!gap) {
          grid.setState(GridIndex3D{x, y, z}, wall ? ObservedVoxelState::kOccupied
                                                   : ObservedVoxelState::kFree);
        }
      }
    }
  }
  SurveyConfig levels = config();
  levels.volume_maximum = Point3{16.0, 12.0, 10.0};
  levels.envelope =
      FlightEnvelopeConfig{.minimum_target_z_m = 0.5, .maximum_target_z_m = 9.5};
  const SurveyReading reading = readSurvey(grid, Point3{6.0, 6.0, 7.0}, levels, {}, {});
  EXPECT_EQ(reading.clusters, 1U);
  EXPECT_EQ(reading.clusters_without_vantage, 1U);
  EXPECT_TRUE(reading.frontiers.empty());
  EXPECT_FALSE(lineOfSight(grid, Point3{4.0, 5.6, 6.75}, Point3{2.375, 5.6, 2.9}));
}

TEST(SurveyMissionTest, TheVolumeFaceIsNotAFrontier) {
  ObservedOccupancyGrid3D grid{GridBounds3D{.origin_x = 0.0,
                                            .origin_y = 0.0,
                                            .origin_z = 0.0,
                                            .resolution_m = kResolution,
                                            .width_cells = 32,
                                            .height_cells = 32,
                                            .depth_cells = 32}};
  for (int z = 0; z < 16; ++z) {
    for (int y = 0; y < 16; ++y) {
      for (int x = 0; x < 16; ++x) {
        grid.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree);
      }
    }
  }
  SurveyConfig volume = config();
  volume.volume_maximum = Point3{4.0, 4.0, 4.0};
  volume.envelope =
      FlightEnvelopeConfig{.minimum_target_z_m = 0.5, .maximum_target_z_m = 3.5};
  const SurveyReading reading = readSurvey(grid, Point3{2.0, 2.0, 2.0}, volume, {}, {});
  EXPECT_EQ(reading.frontier_cells, 0U);
  EXPECT_TRUE(reading.complete());
}

// A volume whose inside is unseen, beside an observed corridor outside it: the
// unknown between the corridor and the volume is a frontier (it leads into the
// volume), the unknown on the corridor's far side is none (it leads away), and
// the frontier has a vantage in the corridor.
TEST(SurveyMissionTest, AFrontierOutsideTheVolumeIsReadWhenItLeadsIntoIt) {
  ObservedOccupancyGrid3D grid{GridBounds3D{.origin_x = 0.0,
                                            .origin_y = 0.0,
                                            .origin_z = 0.0,
                                            .resolution_m = kResolution,
                                            .width_cells = 32,
                                            .height_cells = 32,
                                            .depth_cells = 32}};
  // The corridor: x from 5 to 7 m, the whole length in y, 4 m high, observed free.
  for (int z = 0; z < 16; ++z) {
    for (int y = 0; y < 32; ++y) {
      for (int x = 20; x < 28; ++x) {
        grid.setState(GridIndex3D{x, y, z}, ObservedVoxelState::kFree);
      }
    }
  }
  SurveyConfig volume = config();
  volume.volume_minimum = Point3{0.0, 0.0, 0.0};
  volume.volume_maximum = Point3{4.0, 8.0, 4.0};
  volume.envelope =
      FlightEnvelopeConfig{.minimum_target_z_m = 0.5, .maximum_target_z_m = 3.5};
  const SurveyReading reading = readSurvey(grid, Point3{6.0, 4.0, 2.0}, volume, {}, {});
  EXPECT_TRUE(reading.complete());
  EXPECT_TRUE(reading.frontier_set.contains(GridIndex3D{20, 10, 8}));
  EXPECT_FALSE(reading.frontier_set.contains(GridIndex3D{27, 10, 8}));
  EXPECT_FALSE(reading.frontier_set.contains(GridIndex3D{24, 10, 8}));
  ASSERT_FALSE(reading.frontiers.empty());
  EXPECT_GT(reading.frontiers.front().vantage.x, 5.0);
  EXPECT_LT(reading.frontiers.front().vantage.x, 7.0);
}

// The volume is the level under a hall's floor, reached through an opening in
// it: the hall's cells over the opening, outside the volume, are a frontier
// (the unknown below leads into the volume) with a vantage in the hall.
TEST(SurveyMissionTest, AnOpeningInTheFloorLeadsIntoTheVolumeBelow) {
  ObservedOccupancyGrid3D grid{GridBounds3D{.origin_x = 0.0,
                                            .origin_y = 0.0,
                                            .origin_z = 0.0,
                                            .resolution_m = kResolution,
                                            .width_cells = 32,
                                            .height_cells = 32,
                                            .depth_cells = 32}};
  // The hall: 8 x 8 m, from 3.5 to 7 m up, observed free; its floor from 3 to
  // 3.5 m occupied but for an opening of 2 x 2 m in the middle, unseen.
  for (int z = 12; z < 28; ++z) {
    for (int y = 0; y < 32; ++y) {
      for (int x = 0; x < 32; ++x) {
        const bool floor = z < 14;
        const bool opening = x >= 12 && x < 20 && y >= 12 && y < 20;
        if (floor && opening) {
          continue;
        }
        grid.setState(GridIndex3D{x, y, z}, floor ? ObservedVoxelState::kOccupied
                                                  : ObservedVoxelState::kFree);
      }
    }
  }
  SurveyConfig volume = config();
  volume.volume_minimum = Point3{0.0, 0.0, 0.0};
  volume.volume_maximum = Point3{8.0, 8.0, 3.0};
  volume.envelope =
      FlightEnvelopeConfig{.minimum_target_z_m = 0.5, .maximum_target_z_m = 6.8};
  const SurveyReading reading = readSurvey(grid, Point3{4.0, 4.0, 5.0}, volume, {}, {});
  EXPECT_TRUE(reading.complete());
  EXPECT_TRUE(reading.frontier_set.contains(GridIndex3D{15, 15, 14}));
  EXPECT_FALSE(reading.frontier_set.contains(GridIndex3D{15, 15, 20}));
  ASSERT_FALSE(reading.frontiers.empty());
  EXPECT_GT(reading.frontiers.front().vantage.z, 4.9);
}

} // namespace
} // namespace drone_city_nav
