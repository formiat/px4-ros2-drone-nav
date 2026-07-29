#include "drone_city_nav/semantic_portal_occupancy.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] KnownPassageMap passageMap() {
  return KnownPassageMap{
      .frame_id = "map",
      .structures =
          {
              PassageStructure{
                  .id = "connector",
                  .center = Point2{10.0, 10.0},
                  .size_x_m = 2.0,
                  .size_y_m = 20.0,
                  .z_min_m = 0.0,
                  .z_max_m = 20.0,
                  .openings =
                      {
                          PassageOpening{
                              .id = "portal",
                              .structure_id = "connector",
                              .center = Point3{10.0, 10.0, 10.0},
                              .normal_xy = Point2{1.0, 0.0},
                              .width_m = 6.0,
                              .height_m = 8.0,
                              .depth_m = 2.0,
                              .min_z_m = 6.0,
                              .max_z_m = 14.0,
                              .approach_distance_m = 5.0,
                              .exit_distance_m = 5.0,
                          },
                      },
              },
          },
  };
}

TEST(SemanticPortalOccupancyTest, MarksSideMassesAndKeepsPortalFootprintOpen) {
  OccupancyGrid2D grid{GridBounds{0.0, 0.0, 0.5, 40, 40}};
  grid.reset(CellState::kFree);

  const SemanticPortalOccupancyResult result =
      overlaySemanticPortalSideSolids(grid, passageMap());

  ASSERT_GT(result.cells_marked_occupied, 0U);
  EXPECT_EQ(result.side_volumes_considered, 2U);
  const std::optional<GridIndex> opening_cell = grid.worldToCell(Point2{10.0, 10.0});
  const std::optional<GridIndex> left_cell = grid.worldToCell(Point2{10.0, 16.0});
  const std::optional<GridIndex> right_cell = grid.worldToCell(Point2{10.0, 4.0});
  ASSERT_TRUE(opening_cell.has_value());
  ASSERT_TRUE(left_cell.has_value());
  ASSERT_TRUE(right_cell.has_value());
  EXPECT_FALSE(grid.isOccupied(opening_cell.value_or(GridIndex{})));
  EXPECT_TRUE(grid.isOccupied(left_cell.value_or(GridIndex{})));
  EXPECT_TRUE(grid.isOccupied(right_cell.value_or(GridIndex{})));
}

} // namespace
} // namespace drone_city_nav
