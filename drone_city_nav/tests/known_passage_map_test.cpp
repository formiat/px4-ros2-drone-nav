#include "drone_city_nav/known_passage_map.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace drone_city_nav {
namespace {

[[nodiscard]] std::filesystem::path writePassageMapFixture(const std::string& name,
                                                           const std::string& content) {
  const std::filesystem::path path = std::filesystem::temp_directory_path() /
                                     ("drone_city_nav_" + name + ".passages3d");
  std::ofstream output{path};
  output << content;
  return path;
}

void expectRejected(const std::string& name, const std::string& content) {
  const std::filesystem::path path = writePassageMapFixture(name, content);
  EXPECT_THROW(loadKnownPassageMap(path), std::runtime_error);
  std::filesystem::remove(path);
}

} // namespace

TEST(KnownPassageMap, LoadsValidMapWithStructureAndOpening) {
  const std::filesystem::path path = writePassageMapFixture(
      "valid", "drone_city_nav_known_passages_v1\n"
               "frame_id map\n"
               "structure arch_01 120.0 180.0 14.0 8.0 0.0 40.0\n"
               "opening arch_01 main 120.0 180.0 12.0 2.0 0.0 "
               "8.0 5.0 4.0 9.5 14.5 18.0 20.0\n");

  const KnownPassageMap map = loadKnownPassageMap(path);

  ASSERT_EQ(map.frame_id, "map");
  ASSERT_EQ(map.structures.size(), 1U);
  const PassageStructure& structure = map.structures.front();
  EXPECT_EQ(structure.id, "arch_01");
  EXPECT_DOUBLE_EQ(structure.center.x, 120.0);
  EXPECT_DOUBLE_EQ(structure.size_x_m, 14.0);
  ASSERT_EQ(structure.openings.size(), 1U);
  const PassageOpening& opening = structure.openings.front();
  EXPECT_EQ(opening.id, "main");
  EXPECT_EQ(opening.structure_id, "arch_01");
  EXPECT_DOUBLE_EQ(opening.center.z, 12.0);
  EXPECT_DOUBLE_EQ(opening.normal_xy.x, 1.0);
  EXPECT_DOUBLE_EQ(opening.normal_xy.y, 0.0);
  EXPECT_DOUBLE_EQ(opening.width_m, 8.0);
  EXPECT_DOUBLE_EQ(opening.approach_distance_m, 18.0);
  EXPECT_DOUBLE_EQ(opening.exit_distance_m, 20.0);
  std::filesystem::remove(path);
}

TEST(KnownPassageMap, LoadsEmptyValidMapAndIgnoresComments) {
  const std::filesystem::path path =
      writePassageMapFixture("empty", "# known passages\n"
                                      "drone_city_nav_known_passages_v1\n"
                                      "\n"
                                      "frame_id map # inline comment\n");

  const KnownPassageMap map = loadKnownPassageMap(path);

  EXPECT_EQ(map.frame_id, "map");
  EXPECT_TRUE(map.structures.empty());
  std::filesystem::remove(path);
}

TEST(KnownPassageMap, RejectsDuplicateStructureId) {
  expectRejected("duplicate_structure", "drone_city_nav_known_passages_v1\n"
                                        "frame_id map\n"
                                        "structure arch 10.0 10.0 4.0 4.0 0.0 20.0\n"
                                        "structure arch 12.0 10.0 4.0 4.0 0.0 20.0\n");
}

TEST(KnownPassageMap, RejectsDuplicateOpeningIdWithinStructure) {
  expectRejected("duplicate_opening",
                 "drone_city_nav_known_passages_v1\n"
                 "frame_id map\n"
                 "structure arch 10.0 10.0 10.0 10.0 0.0 20.0\n"
                 "opening arch main 10.0 10.0 8.0 1.0 0.0 4.0 3.0 2.0 "
                 "6.0 10.0 5.0 5.0\n"
                 "opening arch main 10.0 10.0 8.0 1.0 0.0 4.0 3.0 2.0 "
                 "6.0 10.0 5.0 5.0\n");
}

TEST(KnownPassageMap, RejectsOpeningBeforeReferencedStructureExists) {
  expectRejected("unknown_structure",
                 "drone_city_nav_known_passages_v1\n"
                 "frame_id map\n"
                 "opening arch main 10.0 10.0 8.0 1.0 0.0 4.0 3.0 2.0 "
                 "6.0 10.0 5.0 5.0\n");
}

TEST(KnownPassageMap, RejectsInvalidDimensionsAndNormals) {
  expectRejected("bad_structure_size", "drone_city_nav_known_passages_v1\n"
                                       "frame_id map\n"
                                       "structure arch 10.0 10.0 0.0 4.0 0.0 20.0\n");
  expectRejected("bad_opening_width",
                 "drone_city_nav_known_passages_v1\n"
                 "frame_id map\n"
                 "structure arch 10.0 10.0 10.0 10.0 0.0 20.0\n"
                 "opening arch main 10.0 10.0 8.0 1.0 0.0 0.0 3.0 2.0 "
                 "6.0 10.0 5.0 5.0\n");
  expectRejected("bad_opening_normal",
                 "drone_city_nav_known_passages_v1\n"
                 "frame_id map\n"
                 "structure arch 10.0 10.0 10.0 10.0 0.0 20.0\n"
                 "opening arch main 10.0 10.0 8.0 0.0 0.0 4.0 3.0 2.0 "
                 "6.0 10.0 5.0 5.0\n");
}

TEST(KnownPassageMap, RejectsOpeningOutsideStructureOrZRange) {
  expectRejected("outside_footprint",
                 "drone_city_nav_known_passages_v1\n"
                 "frame_id map\n"
                 "structure arch 10.0 10.0 4.0 4.0 0.0 20.0\n"
                 "opening arch main 20.0 10.0 8.0 1.0 0.0 4.0 3.0 2.0 "
                 "6.0 10.0 5.0 5.0\n");
  expectRejected("outside_z_range",
                 "drone_city_nav_known_passages_v1\n"
                 "frame_id map\n"
                 "structure arch 10.0 10.0 10.0 10.0 5.0 20.0\n"
                 "opening arch main 10.0 10.0 4.0 1.0 0.0 4.0 3.0 2.0 "
                 "0.0 10.0 5.0 5.0\n");
}

TEST(KnownPassageMap, RejectsUnknownKeywordAndTrailingToken) {
  expectRejected("unknown_keyword", "drone_city_nav_known_passages_v1\n"
                                    "frame_id map\n"
                                    "passage nope\n");
  expectRejected("trailing_token", "drone_city_nav_known_passages_v1\n"
                                   "frame_id map extra\n");
}

TEST(KnownPassageMap, SourceResultReportsDisabledLoadedFailedAndFrameMismatch) {
  const std::filesystem::path path =
      writePassageMapFixture("source", "drone_city_nav_known_passages_v1\n"
                                       "frame_id map\n"
                                       "structure arch 10.0 10.0 10.0 10.0 0.0 20.0\n");

  KnownPassageSourceConfig config;
  config.enabled = false;
  config.configured_path = path;
  EXPECT_EQ(loadKnownPassageMapSource(config).status,
            KnownPassageSourceStatus::kDisabled);

  config.enabled = true;
  KnownPassageSourceResult result = loadKnownPassageMapSource(config);
  EXPECT_EQ(result.status, KnownPassageSourceStatus::kLoaded);
  EXPECT_TRUE(result.map.has_value());
  EXPECT_TRUE(result.frame_matches);
  EXPECT_EQ(result.structures, 1U);
  EXPECT_EQ(result.openings, 0U);

  config.expected_frame_id = "odom";
  result = loadKnownPassageMapSource(config);
  EXPECT_EQ(result.status, KnownPassageSourceStatus::kLoaded);
  EXPECT_FALSE(result.frame_matches);

  config.configured_path = path.parent_path() / "missing.passages3d";
  result = loadKnownPassageMapSource(config);
  EXPECT_EQ(result.status, KnownPassageSourceStatus::kLoadFailed);
  EXPECT_FALSE(result.map.has_value());
  EXPECT_FALSE(result.error_message.empty());

  std::filesystem::remove(path);
}

TEST(KnownPassageMap, SourceStatusNamesAreStable) {
  EXPECT_STREQ(knownPassageSourceStatusName(KnownPassageSourceStatus::kDisabled),
               "disabled");
  EXPECT_STREQ(knownPassageSourceStatusName(KnownPassageSourceStatus::kLoaded),
               "loaded");
  EXPECT_STREQ(knownPassageSourceStatusName(KnownPassageSourceStatus::kLoadFailed),
               "load_failed");
}

} // namespace drone_city_nav
