#include "drone_city_nav/indexed_point_cloud_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <random>
#include <span>
#include <tuple>
#include <vector>

namespace drone_city_nav {
namespace {

using Coordinates = std::tuple<double, double, double>;

[[nodiscard]] std::vector<Coordinates>
sortedCoordinates(const std::span<const Point3> points) {
  std::vector<Coordinates> coordinates;
  coordinates.reserve(points.size());
  for (const Point3& point : points) {
    coordinates.emplace_back(point.x, point.y, point.z);
  }
  std::ranges::sort(coordinates);
  return coordinates;
}

[[nodiscard]] std::vector<Point3> linearInsideBox(const std::vector<Point3>& points,
                                                  const Point3& minimum,
                                                  const Point3& maximum) {
  std::vector<Point3> inside;
  for (const Point3& point : points) {
    if (point.x >= minimum.x && point.x <= maximum.x && point.y >= minimum.y &&
        point.y <= maximum.y && point.z >= minimum.z && point.z <= maximum.z) {
      inside.push_back(point);
    }
  }
  return inside;
}

TEST(IndexedPointCloud3DTest, ABoxQueryReturnsExactlyTheLinearScansPoints) {
  // Thirty thousand returns spread over a hall, queried with boxes the size a
  // swept horizon segment asks for: the cells answer with the same points the
  // whole-scan pass finds, including points on cell boundaries.
  std::mt19937 generator{7U};
  std::uniform_real_distribution<double> coordinate{-40.0, 40.0};
  std::vector<Point3> points;
  points.reserve(30'000U);
  for (std::size_t index = 0U; index < 30'000U; ++index) {
    points.push_back(Point3{coordinate(generator), coordinate(generator),
                            0.25 * std::floor(coordinate(generator) / 0.25)});
  }
  const IndexedPointCloud3D cloud{points, 2.0};
  const IndexedPointCloudView3D view = cloud.view();
  ASSERT_TRUE(view.indexed());
  EXPECT_EQ(view.points().size(), points.size());
  EXPECT_EQ(sortedCoordinates(view.points()), sortedCoordinates(points));

  std::uniform_real_distribution<double> center{-38.0, 38.0};
  for (int query = 0; query < 200; ++query) {
    const Point3 minimum{center(generator), center(generator), center(generator)};
    const Point3 maximum{minimum.x + 2.6, minimum.y + 2.6, minimum.z + 1.4};
    std::vector<Point3> indexed;
    view.collectInsideBox(minimum, maximum, indexed);
    EXPECT_EQ(sortedCoordinates(indexed),
              sortedCoordinates(linearInsideBox(points, minimum, maximum)))
        << "query " << query;
  }
  // A box wider than the cloud is answered by the same linear pass.
  std::vector<Point3> everything;
  view.collectInsideBox(Point3{-100.0, -100.0, -100.0}, Point3{100.0, 100.0, 100.0},
                        everything);
  EXPECT_EQ(everything.size(), points.size());
}

TEST(IndexedPointCloud3DTest, AnUnindexedViewAnswersWithOneLinearPass) {
  const std::array points{Point3{0.5, 0.5, 0.5}, Point3{3.0, 0.5, 0.5},
                          Point3{0.5, 3.0, 0.5}};
  const IndexedPointCloudView3D view{points};
  EXPECT_FALSE(view.indexed());
  EXPECT_EQ(view.points().size(), 3U);
  std::vector<Point3> inside;
  view.collectInsideBox(Point3{0.0, 0.0, 0.0}, Point3{1.0, 1.0, 1.0}, inside);
  ASSERT_EQ(inside.size(), 1U);
  EXPECT_DOUBLE_EQ(inside.front().x, 0.5);

  const IndexedPointCloudView3D empty;
  EXPECT_TRUE(empty.empty());
  inside.clear();
  empty.collectInsideBox(Point3{-1.0, -1.0, -1.0}, Point3{1.0, 1.0, 1.0}, inside);
  EXPECT_TRUE(inside.empty());
}

TEST(IndexedPointCloud3DTest, AnEmptyOrDegenerateCloudCarriesNoCells) {
  const IndexedPointCloud3D empty{std::vector<Point3>{}, 2.0};
  EXPECT_FALSE(empty.view().indexed());
  EXPECT_TRUE(empty.view().empty());
  const IndexedPointCloud3D unsized{std::vector<Point3>{Point3{1.0, 1.0, 1.0}}, 0.0};
  EXPECT_FALSE(unsized.view().indexed());
  EXPECT_EQ(unsized.view().points().size(), 1U);
  std::vector<Point3> inside;
  unsized.view().collectInsideBox(Point3{0.0, 0.0, 0.0}, Point3{2.0, 2.0, 2.0}, inside);
  EXPECT_EQ(inside.size(), 1U);
}

} // namespace
} // namespace drone_city_nav
