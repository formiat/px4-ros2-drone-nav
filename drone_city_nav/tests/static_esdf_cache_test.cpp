#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/occupancy_grid_3d.hpp"
#include "drone_city_nav/static_esdf_cache.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace drone_city_nav {
namespace {

class TemporaryCacheFile {
public:
  TemporaryCacheFile() {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    path_ = std::filesystem::temp_directory_path() /
            ("drone_city_nav_static_esdf_" + std::to_string(suffix) + ".bin");
  }

  ~TemporaryCacheFile() {
    std::filesystem::remove(path_);
  }

  TemporaryCacheFile(const TemporaryCacheFile&) = delete;
  TemporaryCacheFile& operator=(const TemporaryCacheFile&) = delete;
  TemporaryCacheFile(TemporaryCacheFile&&) = delete;
  TemporaryCacheFile& operator=(TemporaryCacheFile&&) = delete;

  [[nodiscard]] const std::filesystem::path& path() const noexcept {
    return path_;
  }

private:
  std::filesystem::path path_;
};

[[nodiscard]] GridBounds3D testBounds() {
  return GridBounds3D{.origin_x = -2.0,
                      .origin_y = 4.0,
                      .origin_z = 0.0,
                      .resolution_m = 0.5,
                      .width_cells = 20,
                      .height_cells = 18,
                      .depth_cells = 12};
}

[[nodiscard]] OccupancyGrid3D testOccupancy(const std::uint64_t fingerprint = 42U) {
  OccupancyGrid3D occupancy{testBounds(), fingerprint};
  occupancy.setOccupied(GridIndex3D{1, 2, 3});
  occupancy.setOccupied(GridIndex3D{17, 14, 9});
  occupancy.setOccupied(GridIndex3D{9, 8, 6});
  return occupancy;
}

TEST(StaticEsdfCache, RoundTripPreservesGlobalDistanceField) {
  const OccupancyGrid3D occupancy = testOccupancy();
  const DistanceField3D expected = DistanceField3D::build(occupancy, 6.0);
  TemporaryCacheFile file;
  StaticEsdfCache::write(file.path(), occupancy, expected, 1);

  const StaticEsdfCache cache = StaticEsdfCache::load(file.path());
  ASSERT_TRUE(cache.compatibleWith(occupancy, 6.0));
  EXPECT_EQ(cache.occupancyFingerprint(), occupancy.fingerprint());
  EXPECT_DOUBLE_EQ(cache.maximumDistanceM(), 6.0);
  EXPECT_GT(cache.storedChunkCount(), 0U);
  const StaticEsdfCacheExtraction extracted = cache.extract(testBounds(), 6.0);
  ASSERT_EQ(extracted.field.distancesM().size(), expected.distancesM().size());
  for (std::size_t index = 0U; index < expected.distancesM().size(); ++index) {
    EXPECT_FLOAT_EQ(extracted.field.distancesM()[index], expected.distancesM()[index]);
  }
  EXPECT_GT(extracted.stats.decoded_chunks, 0U);
  EXPECT_EQ(extracted.stats.requested_chunks, extracted.stats.decoded_chunks);
  EXPECT_EQ(extracted.stats.decoded_chunk_cache_hits, 0U);
  EXPECT_GT(extracted.stats.finite_voxels, 0U);
}

TEST(StaticEsdfCache, LoadedInstancesShareDecodedChunks) {
  const OccupancyGrid3D occupancy = testOccupancy();
  const DistanceField3D global = DistanceField3D::build(occupancy, 6.0);
  TemporaryCacheFile file;
  StaticEsdfCache::write(file.path(), occupancy, global, 1);

  const StaticEsdfCache first = StaticEsdfCache::load(file.path());
  const StaticEsdfCache second = StaticEsdfCache::load(file.path());
  EXPECT_FALSE(first.sharedResourceReused());
  EXPECT_TRUE(second.sharedResourceReused());

  const StaticEsdfCacheExtraction initial = first.extract(testBounds(), 6.0);
  ASSERT_GT(initial.stats.decoded_chunks, 0U);
  const StaticEsdfCacheExtraction reused = second.extract(testBounds(), 6.0);
  EXPECT_EQ(reused.stats.decoded_chunks, 0U);
  EXPECT_EQ(reused.stats.decoded_chunk_cache_hits, reused.stats.requested_chunks);
  EXPECT_EQ(reused.stats.resident_decoded_chunks,
            initial.stats.resident_decoded_chunks);
  ASSERT_EQ(reused.field.distancesM().size(), initial.field.distancesM().size());
  for (std::size_t index = 0U; index < initial.field.distancesM().size(); ++index) {
    EXPECT_FLOAT_EQ(reused.field.distancesM()[index],
                    initial.field.distancesM()[index]);
  }
}

TEST(StaticEsdfCache, AlignsRegionsToStableChunkBoundaries) {
  const GridBounds3D requested{.origin_x = -1.0,
                               .origin_y = 5.5,
                               .origin_z = 1.0,
                               .resolution_m = 0.5,
                               .width_cells = 15,
                               .height_cells = 12,
                               .depth_cells = 8};
  const GridBounds3D aligned =
      StaticEsdfCache::alignRegionToChunks(testBounds(), requested);

  EXPECT_DOUBLE_EQ(aligned.origin_x, -2.0);
  EXPECT_DOUBLE_EQ(aligned.origin_y, 4.0);
  EXPECT_DOUBLE_EQ(aligned.origin_z, 0.0);
  EXPECT_EQ(aligned.width_cells, 20);
  EXPECT_EQ(aligned.height_cells, 16);
  EXPECT_EQ(aligned.depth_cells, 12);
}

TEST(StaticEsdfCache, ExtractsAlignedLocalRegionAndAppliesSmallerCap) {
  const OccupancyGrid3D occupancy = testOccupancy();
  const DistanceField3D global = DistanceField3D::build(occupancy, 6.0);
  TemporaryCacheFile file;
  StaticEsdfCache::write(file.path(), occupancy, global, 1);
  const StaticEsdfCache cache = StaticEsdfCache::load(file.path());
  const GridBounds3D local{.origin_x = -1.0,
                           .origin_y = 5.5,
                           .origin_z = 1.0,
                           .resolution_m = 0.5,
                           .width_cells = 15,
                           .height_cells = 12,
                           .depth_cells = 8};
  const DistanceField3D extracted = cache.extract(local, 2.0).field;

  for (int z = 0; z < local.depth_cells; ++z) {
    for (int y = 0; y < local.height_cells; ++y) {
      for (int x = 0; x < local.width_cells; ++x) {
        const float expected = global.distanceAt(GridIndex3D{x + 2, y + 3, z + 2});
        const float actual = extracted.distanceAt(GridIndex3D{x, y, z});
        if (std::isfinite(expected) && expected <= 2.0F) {
          EXPECT_FLOAT_EQ(actual, expected);
        } else {
          EXPECT_TRUE(std::isinf(actual));
        }
      }
    }
  }
}

TEST(StaticEsdfCache, RejectsMismatchedWorldAndUnsupportedDistance) {
  const OccupancyGrid3D occupancy = testOccupancy();
  const DistanceField3D global = DistanceField3D::build(occupancy, 3.0);
  TemporaryCacheFile file;
  StaticEsdfCache::write(file.path(), occupancy, global, 1);
  const StaticEsdfCache cache = StaticEsdfCache::load(file.path());

  const OccupancyGrid3D different_world = testOccupancy(43U);
  EXPECT_FALSE(cache.compatibleWith(different_world, 3.0));
  EXPECT_FALSE(cache.compatibleWith(occupancy, 3.5));
  EXPECT_THROW(static_cast<void>(cache.extract(testBounds(), 3.5)),
               std::invalid_argument);
}

TEST(StaticEsdfCache, DetectsCorruptedChunkAtExtraction) {
  const OccupancyGrid3D occupancy = testOccupancy();
  const DistanceField3D global = DistanceField3D::build(occupancy, 6.0);
  TemporaryCacheFile file;
  StaticEsdfCache::write(file.path(), occupancy, global, 1);
  {
    std::fstream stream{file.path(), std::ios::binary | std::ios::in | std::ios::out};
    ASSERT_TRUE(stream);
    stream.seekg(-1, std::ios::end);
    char value{};
    stream.read(&value, 1);
    value = static_cast<char>(value ^ 0x5A);
    stream.seekp(-1, std::ios::end);
    stream.write(&value, 1);
  }
  const StaticEsdfCache cache = StaticEsdfCache::load(file.path());
  EXPECT_THROW(static_cast<void>(cache.extract(testBounds(), 6.0)), std::runtime_error);
}

TEST(StaticEsdfCache, LoadsCommittedCompactPassageFixture) {
  const OccupancyGrid3D occupancy =
      OccupancyGrid3D::load(TEST_COMPACT_PASSAGE_OCCUPANCY3D_PATH);
  const StaticEsdfCache cache = StaticEsdfCache::load(TEST_COMPACT_PASSAGE_ESDF3D_PATH);

  ASSERT_TRUE(cache.compatibleWith(occupancy, 8.0));
  EXPECT_GT(cache.storedChunkCount(), 0U);
  const StaticEsdfCacheExtraction extraction = cache.extract(occupancy.bounds(), 8.0);
  EXPECT_EQ(extraction.field.distancesM().size(), 349440U);
  EXPECT_GT(extraction.stats.finite_voxels, 0U);
}

} // namespace
} // namespace drone_city_nav
