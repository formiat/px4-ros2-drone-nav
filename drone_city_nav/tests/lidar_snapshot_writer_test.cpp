#include "drone_city_nav/lidar_snapshot_writer.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace drone_city_nav {

TEST(LidarSnapshotWriter, JsonEscapesStrings) {
  LidarSnapshotRecord record;
  record.snapshot = "snap\"one";
  record.yaw_source = "px4\nheading";
  record.image_path = "/tmp/image\\path.ppm";
  record.scan_csv_path = "/tmp/scan.csv";

  std::ostringstream stream;
  writeLidarSnapshotSummary(stream, record);

  const std::string json = stream.str();
  EXPECT_NE(json.find("\"snapshot\":\"snap\\\"one\""), std::string::npos);
  EXPECT_NE(json.find("\"yaw_source\":\"px4\\nheading\""), std::string::npos);
  EXPECT_NE(json.find("/tmp/image\\\\path.ppm"), std::string::npos);
}

TEST(LidarSnapshotWriter, WritesFiniteNumbersAndNullForNonFinite) {
  LidarSnapshotRecord record;
  record.snapshot = "snapshot";
  record.yaw_delta_to_attitude_rad = std::numeric_limits<double>::quiet_NaN();
  record.scan_receive_age_s = 1.25;
  record.image_path = "/tmp/image.ppm";
  record.scan_csv_path = "/tmp/scan.csv";

  std::ostringstream stream;
  writeLidarSnapshotSummary(stream, record);

  const std::string json = stream.str();
  EXPECT_NE(json.find("\"yaw_delta_to_attitude_rad\":null"), std::string::npos);
  EXPECT_NE(json.find("\"scan_receive_age_s\":1.2500"), std::string::npos);
}

TEST(LidarSnapshotWriter, CsvContainsProjectionStatusAndHitCoordinates) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "drone_city_nav_lidar_scan.csv";
  LidarSnapshotCsvRow row;
  row.beam_index = 7U;
  row.angle_rad = 0.5;
  row.raw_range_m = 4.0;
  row.used_range_m = 4.0;
  row.hit = true;
  row.end_x_m = 1.0;
  row.end_y_m = 2.0;
  row.status = LidarBeamProjectionStatus::kAccepted;
  row.lidar_direction = Point3{1.0, 0.0, 0.0};

  ASSERT_TRUE(writeLidarScanCsv(path, std::vector<LidarSnapshotCsvRow>{row}));

  std::ifstream input{path};
  ASSERT_TRUE(input.is_open());
  const std::string content{std::istreambuf_iterator<char>{input},
                            std::istreambuf_iterator<char>{}};
  EXPECT_NE(content.find("beam_index,angle_rad"), std::string::npos);
  EXPECT_NE(content.find("7,0.5,4,4,1,1,2"), std::string::npos);
  EXPECT_NE(content.find(",accepted,"), std::string::npos);

  std::error_code ignored;
  std::filesystem::remove(path, ignored);
}

TEST(LidarSnapshotWriter, EmptyHitsStillWritesValidJsonArrayAndCsvHeader) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "drone_city_nav_lidar_empty.csv";
  ASSERT_TRUE(writeLidarScanCsv(path, {}));

  std::ifstream input{path};
  ASSERT_TRUE(input.is_open());
  const std::string content{std::istreambuf_iterator<char>{input},
                            std::istreambuf_iterator<char>{}};
  EXPECT_NE(content.find("beam_index,angle_rad"), std::string::npos);

  LidarSnapshotRecord record;
  record.snapshot = "empty";
  record.image_path = "/tmp/image.ppm";
  record.scan_csv_path = path;
  std::ostringstream stream;
  writeLidarSnapshotSummary(stream, record);
  EXPECT_NE(stream.str().find("\"hit_points\":[]"), std::string::npos);

  std::error_code ignored;
  std::filesystem::remove(path, ignored);
}

} // namespace drone_city_nav
