#include "drone_city_nav/regional_topology_graph_3d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <optional>
#include <ranges>
#include <span>
#include <unordered_set>

namespace drone_city_nav {
namespace {

[[nodiscard]] IncrementalTopologyGraph3DConfig makeConfig() {
  IncrementalTopologyGraph3DConfig config;
  config.block_size_cells = 4;
  config.coarse_sample_stride_cells = 1;
  config.refined_sample_stride_cells = 1;
  config.maximum_observed_blocks_per_update = 1024U;
  config.minimum_oldest_blocks_per_update = 0U;
  config.footprint.radius_m = 0.2;
  config.footprint.lower_extent_m = 0.2;
  config.footprint.upper_extent_m = 0.2;
  config.footprint.perimeter_samples = 8U;
  config.footprint.radial_rings = 1U;
  config.footprint.axial_samples = 2U;
  config.footprint.sweep_step_m = 0.25;
  config.require_known_free_space = true;
  return config;
}

void fillStateBox(ObservedOccupancyGrid3D& occupancy, const int minimum_x,
                  const int maximum_x, const int minimum_y, const int maximum_y,
                  const int minimum_z, const int maximum_z,
                  const ObservedVoxelState state) {
  for (int z = minimum_z; z <= maximum_z; ++z) {
    for (int y = minimum_y; y <= maximum_y; ++y) {
      for (int x = minimum_x; x <= maximum_x; ++x) {
        static_cast<void>(occupancy.setState({x, y, z}, state));
        ASSERT_EQ(occupancy.state({x, y, z}), state);
      }
    }
  }
}

void fillFreeBox(ObservedOccupancyGrid3D& occupancy, const int minimum_x,
                 const int maximum_x, const int minimum_y, const int maximum_y,
                 const int minimum_z, const int maximum_z) {
  fillStateBox(occupancy, minimum_x, maximum_x, minimum_y, maximum_y, minimum_z,
               maximum_z, ObservedVoxelState::kFree);
}

void fillOccupied(ObservedOccupancyGrid3D& occupancy) {
  const GridBounds3D& bounds = occupancy.bounds();
  fillStateBox(occupancy, 0, bounds.width_cells - 1, 0, bounds.height_cells - 1, 0,
               bounds.depth_cells - 1, ObservedVoxelState::kOccupied);
}

TEST(RegionalTopologyGraph3DTest,
     ContractsStraightCorridorIntoOneCertifiedRegionalEdge) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 45, 9, 11, 5, 7);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot source_snapshot = source.snapshot();

  const RegionalTopologyGraph3D regional =
      buildRegionalTopologyGraph3D(source_snapshot);

  ASSERT_GT(source_snapshot.nodes().size(), 2U);
  ASSERT_EQ(regional.nodes().size(), 2U);
  ASSERT_EQ(regional.edges().size(), 1U);
  const RegionalTopologyEdge3D& edge = regional.edges().front();
  EXPECT_EQ(edge.source_nodes.size(), source_snapshot.nodes().size());
  EXPECT_EQ(edge.source_edges.size(), source_snapshot.edges().size());
  EXPECT_EQ(edge.source_edges.size() + 1U, edge.source_nodes.size());
  EXPECT_GT(edge.length_m, 0.0);
  EXPECT_GE(edge.polyline.size(), 2U);
  EXPECT_EQ(edge.created_on_revision, 1U);
  EXPECT_EQ(edge.evidence.validated_through_revision, 1U);
  EXPECT_TRUE(std::ranges::all_of(
      std::views::iota(std::size_t{1U}, edge.polyline.size()),
      [&](const std::size_t index) {
        return rawSweptFootprintIsNavigable(
            occupancy, edge.polyline[index - 1U], FootprintBodyAxis{},
            edge.polyline[index], FootprintBodyAxis{}, makeConfig().footprint);
      }));
}

TEST(RegionalTopologyGraph3DTest,
     ContractsNonBranchingObservedBoundaryBendsWithoutLosingTheirGeometry) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillFreeBox(occupancy, 2, 24, 9, 11, 5, 7);
  fillFreeBox(occupancy, 22, 24, 9, 45, 5, 7);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot source_snapshot = source.snapshot();

  const RegionalTopologyGraph3D regional =
      buildRegionalTopologyGraph3D(source_snapshot);

  ASSERT_GT(source_snapshot.nodes().size(), 2U);
  EXPECT_TRUE(std::ranges::any_of(source_snapshot.nodes(), [](const auto& node) {
    return node.unknown_boundary_exposure;
  }));
  EXPECT_LT(regional.nodes().size(), source_snapshot.nodes().size());
  std::unordered_set<IncrementalTopologyEdgeId, IncrementalTopologyEdgeIdHash>
      represented;
  bool horizontal_leg = false;
  bool vertical_leg = false;
  for (const RegionalTopologyEdge3D& edge : regional.edges()) {
    horizontal_leg =
        horizontal_leg || std::ranges::any_of(edge.polyline, [](const Point3& point) {
          return point.x > 20.0 && point.y < 15.0;
        });
    vertical_leg =
        vertical_leg || std::ranges::any_of(edge.polyline, [](const Point3& point) {
          return point.x > 20.0 && point.y > 40.0;
        });
    for (const IncrementalTopologyEdgeId source_edge : edge.source_edges) {
      EXPECT_TRUE(represented.insert(source_edge).second);
    }
  }
  EXPECT_EQ(represented.size(), source_snapshot.edges().size());
  EXPECT_TRUE(std::ranges::any_of(regional.nodes(), [](const auto& node) {
    return node.unknown_boundary_exposure;
  }));
  EXPECT_TRUE(horizontal_leg);
  EXPECT_TRUE(vertical_leg);
}

TEST(RegionalTopologyGraph3DTest, ExplicitAnchorSplitsContractedCorridor) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 45, 9, 11, 5, 7);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot source_snapshot = source.snapshot();
  const std::optional<IncrementalTopologyNodeId> anchor =
      source_snapshot.nearestNode({24.5, 10.5, 6.5}, 4.0);
  ASSERT_TRUE(anchor.has_value());
  const IncrementalTopologyNodeId anchor_id =
      anchor.value_or(IncrementalTopologyNodeId{});
  const IncrementalTopologyNode3D* source_anchor = source_snapshot.findNode(anchor_id);
  ASSERT_NE(source_anchor, nullptr);
  ASSERT_EQ(source_anchor->degree, 2U);

  const std::array anchors{anchor_id};
  const RegionalTopologyGraph3D regional =
      buildRegionalTopologyGraph3D(source_snapshot, anchors);

  ASSERT_EQ(regional.nodes().size(), 3U);
  ASSERT_EQ(regional.edges().size(), 2U);
  const RegionalTopologyNode3D* regional_anchor = regional.findNode(anchor_id);
  ASSERT_NE(regional_anchor, nullptr);
  EXPECT_EQ(regional_anchor->degree, 2U);
  EXPECT_EQ(std::ranges::count_if(regional.edges(),
                                  [&](const auto& edge) {
                                    return edge.first == anchor_id ||
                                           edge.second == anchor_id;
                                  }),
            2);
  std::size_t represented_source_edges = 0U;
  for (const RegionalTopologyEdge3D& edge : regional.edges()) {
    represented_source_edges += edge.source_edges.size();
  }
  EXPECT_EQ(represented_source_edges, source_snapshot.edges().size());
}

TEST(RegionalTopologyGraph3DTest, ContractsXJunctionAroundSemanticBranch) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 45, 21, 23, 5, 7);
  fillFreeBox(occupancy, 21, 23, 2, 45, 5, 7);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot source_snapshot = source.snapshot();

  const RegionalTopologyGraph3D regional =
      buildRegionalTopologyGraph3D(source_snapshot);

  EXPECT_TRUE(std::ranges::any_of(
      regional.nodes(), [](const auto& node) { return node.traits.junction; }));
  EXPECT_LT(regional.nodes().size(), source_snapshot.nodes().size());
  EXPECT_GE(regional.edges().size(), 4U);
}

TEST(RegionalTopologyGraph3DTest, PreservesClosedLoopWithoutDuplicateSourceEdges) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 48, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 8, 39, 8, 10, 5, 7);
  fillFreeBox(occupancy, 8, 39, 37, 39, 5, 7);
  fillFreeBox(occupancy, 8, 10, 8, 39, 5, 7);
  fillFreeBox(occupancy, 37, 39, 8, 39, 5, 7);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));
  const IncrementalTopologyGraph3DSnapshot source_snapshot = source.snapshot();

  const RegionalTopologyGraph3D regional =
      buildRegionalTopologyGraph3D(source_snapshot);

  ASSERT_GE(regional.nodes().size(), 2U);
  ASSERT_EQ(regional.edges().size(), regional.nodes().size());
  EXPECT_TRUE(std::ranges::all_of(regional.nodes(), [](const auto& node) {
    return node.degree == 2U && !node.traits.terminal;
  }));
  std::unordered_set<IncrementalTopologyEdgeId, IncrementalTopologyEdgeIdHash>
      represented;
  for (const RegionalTopologyEdge3D& edge : regional.edges()) {
    for (const IncrementalTopologyEdgeId source_edge : edge.source_edges) {
      EXPECT_TRUE(represented.insert(source_edge).second);
    }
  }
  EXPECT_EQ(represented.size(), source_snapshot.edges().size());
}

TEST(RegionalTopologyGraph3DTest, PreservesVerticalConnectorAsGraphEvent) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 48, 24, 32}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 22, 9, 11, 3, 5);
  fillFreeBox(occupancy, 20, 22, 9, 11, 3, 24);
  fillFreeBox(occupancy, 20, 45, 9, 11, 22, 24);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));

  const RegionalTopologyGraph3D regional =
      buildRegionalTopologyGraph3D(source.snapshot());

  EXPECT_TRUE(std::ranges::any_of(regional.nodes(), [](const auto& node) {
    return node.traits.vertical_connector;
  }));
}

TEST(RegionalTopologyGraph3DTest, KeepsStableIdentityForUnchangedCorridor) {
  ObservedOccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 64, 32, 16}};
  fillOccupied(occupancy);
  fillFreeBox(occupancy, 2, 61, 21, 23, 5, 7);
  IncrementalTopologyGraph3D source{makeConfig()};
  static_cast<void>(source.update(occupancy, 1U, {}, true));
  const RegionalTopologyGraph3D before =
      buildRegionalTopologyGraph3D(source.snapshot());
  ASSERT_FALSE(before.edges().empty());
  const RegionalTopologyEdgeId3D stable_id = before.edges().front().id;

  fillFreeBox(occupancy, 3, 7, 3, 5, 5, 7);
  const OccupancyChunkIndex3D dirty = ObservedOccupancyGrid3D::chunkIndex({4, 4, 6});
  static_cast<void>(source.update(occupancy, 2U, std::span{&dirty, 1U}, false));
  const RegionalTopologyGraph3D after = buildRegionalTopologyGraph3D(source.snapshot());

  EXPECT_NE(after.findEdge(stable_id), nullptr);
}

} // namespace
} // namespace drone_city_nav
