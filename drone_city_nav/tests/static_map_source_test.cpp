#include "drone_city_nav/static_map_source.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

namespace drone_city_nav {
namespace {

class ScopedCurrentPath {
public:
  explicit ScopedCurrentPath(const std::filesystem::path& path)
      : previous_{std::filesystem::current_path()} {
    std::filesystem::current_path(path);
  }

  ~ScopedCurrentPath() {
    std::filesystem::current_path(previous_);
  }

  ScopedCurrentPath(const ScopedCurrentPath&) = delete;
  ScopedCurrentPath& operator=(const ScopedCurrentPath&) = delete;
  ScopedCurrentPath(ScopedCurrentPath&&) = delete;
  ScopedCurrentPath& operator=(ScopedCurrentPath&&) = delete;

private:
  std::filesystem::path previous_;
};

[[nodiscard]] std::filesystem::path makeTempDir(const std::string& name) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / ("drone_city_nav_" + name);
  std::filesystem::remove_all(path);
  std::filesystem::create_directories(path);
  return path;
}

[[nodiscard]] std::string mapContent(const std::string& frame_id = "map") {
  return "drone_city_nav_static_map_v1\n"
         "frame_id " +
         frame_id +
         "\n"
         "bounds 0.0 0.0 1.0 10.0 10.0\n"
         "rect building_a 5.0 5.0 2.0 2.0 8.0\n";
}

std::filesystem::path writeMapFile(const std::filesystem::path& path,
                                   const std::string& content) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output{path};
  output << content;
  return path;
}

} // namespace

TEST(StaticMapSource, DisabledReturnsResolvedPathWithoutGrid) {
  const std::filesystem::path root = makeTempDir("disabled_source");
  const std::filesystem::path map_path =
      writeMapFile(root / "city.map2d", mapContent());

  const StaticMapSourceResult result =
      loadStaticMapSource(StaticMapSourceConfig{false, map_path, {}, "map", 0.0});

  EXPECT_EQ(result.status, StaticMapSourceStatus::kDisabled);
  EXPECT_EQ(result.resolved_path, map_path);
  EXPECT_FALSE(result.grid.has_value());
  std::filesystem::remove_all(root);
}

TEST(StaticMapSource, KeepsAbsoluteConfiguredPath) {
  const std::filesystem::path absolute_path =
      std::filesystem::temp_directory_path() / "drone_city_nav_absolute.map2d";

  EXPECT_EQ(resolveStaticMapPath(absolute_path, {}), absolute_path);
}

TEST(StaticMapSource, ResolvesExistingRelativePathFromCurrentDirectory) {
  const std::filesystem::path root = makeTempDir("relative_source");
  writeMapFile(root / "local.map2d", mapContent());
  const ScopedCurrentPath cwd{root};

  const std::filesystem::path resolved = resolveStaticMapPath("local.map2d", {});

  EXPECT_EQ(resolved, std::filesystem::absolute(root / "local.map2d"));
  std::filesystem::remove_all(root);
}

TEST(StaticMapSource, ResolvesPackageShareCandidate) {
  const std::filesystem::path root = makeTempDir("package_candidate");
  writeMapFile(root / "maps" / "city.map2d", mapContent());

  const std::filesystem::path resolved = resolveStaticMapPath("maps/city.map2d", root);

  EXPECT_EQ(resolved, root / "maps" / "city.map2d");
  std::filesystem::remove_all(root);
}

TEST(StaticMapSource, ResolvesPackageWorldsFallbackByFilename) {
  const std::filesystem::path root = makeTempDir("worlds_fallback");
  writeMapFile(root / "worlds" / "city.map2d", mapContent());

  const std::filesystem::path resolved =
      resolveStaticMapPath("missing/city.map2d", root);

  EXPECT_EQ(resolved, root / "worlds" / "city.map2d");
  std::filesystem::remove_all(root);
}

TEST(StaticMapSource, LoadsAndRasterizesValidMap) {
  const std::filesystem::path root = makeTempDir("valid_load");
  const std::filesystem::path map_path =
      writeMapFile(root / "city.map2d", mapContent());

  const StaticMapSourceResult result =
      loadStaticMapSource(StaticMapSourceConfig{true, map_path, {}, "map", 0.0});

  EXPECT_EQ(result.status, StaticMapSourceStatus::kLoaded);
  EXPECT_TRUE(result.frame_matches);
  EXPECT_EQ(result.map_frame_id, "map");
  EXPECT_EQ(result.rectangles, 1U);
  ASSERT_TRUE(result.grid.has_value());
  if (!result.grid.has_value()) {
    return;
  }
  const OccupancyGrid2D& grid = *result.grid;
  EXPECT_GT(result.occupied_cells, 0U);
  EXPECT_TRUE(grid.isOccupied(GridIndex{4, 4}));
  std::filesystem::remove_all(root);
}

TEST(StaticMapSource, ReportsFrameMismatchWithoutFailingLoad) {
  const std::filesystem::path root = makeTempDir("frame_mismatch");
  const std::filesystem::path map_path =
      writeMapFile(root / "city.map2d", mapContent("world"));

  const StaticMapSourceResult result =
      loadStaticMapSource(StaticMapSourceConfig{true, map_path, {}, "map", 0.0});

  EXPECT_EQ(result.status, StaticMapSourceStatus::kLoaded);
  EXPECT_FALSE(result.frame_matches);
  EXPECT_EQ(result.map_frame_id, "world");
  EXPECT_TRUE(result.grid.has_value());
  std::filesystem::remove_all(root);
}

TEST(StaticMapSource, ReportsLoadFailureWithErrorMessage) {
  const StaticMapSourceResult result = loadStaticMapSource(StaticMapSourceConfig{
      true, "/tmp/drone_city_nav_missing_static_map_source.map2d", {}, "map", 0.0});

  EXPECT_EQ(result.status, StaticMapSourceStatus::kLoadFailed);
  EXPECT_FALSE(result.grid.has_value());
  EXPECT_FALSE(result.error_message.empty());
}

} // namespace drone_city_nav
