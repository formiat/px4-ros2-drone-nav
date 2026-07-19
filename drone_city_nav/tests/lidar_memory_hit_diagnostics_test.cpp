#include "drone_city_nav/lidar_memory_hit_diagnostics.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>

namespace drone_city_nav {
namespace {

[[nodiscard]] LidarMemoryHitDiagnosticRecord makeRecord() {
  LidarBeamObservation observation{};
  observation.beam_index = 17U;
  observation.acquisition_stamp_ns = 123'456'789;
  observation.acquisition_stamp_valid = true;
  observation.receive_stamp_ns = 123'556'789;
  observation.receive_stamp_valid = true;
  observation.measured_range_m = 6.5;
  observation.effective_max_range_m = 35.0;
  observation.source_attitude_valid = true;
  observation.source_roll_rad = 0.1;
  observation.source_pitch_rad = -0.2;
  observation.source_tilt_rad = 0.3;
  observation.projection = LidarBeamProjection{
      .status = LidarBeamProjectionStatus::kAccepted,
      .hit = true,
      .used_range_m = 6.5,
      .endpoint_altitude_m = 8.0,
      .endpoint = Point2{11.0, 12.0},
      .lidar_direction = Point3{1.0, 0.0, 0.0},
      .body_frd_direction = Point3{1.0, 0.0, 0.0},
      .ned_direction = Point3{1.0, 0.0, 0.0},
      .ray_origin_map_m = Point3{5.0, 6.0, 7.0},
      .ray_direction_map = Point3{1.0, 0.0, 0.0},
      .endpoint_map_m = Point3{11.5, 6.0, 7.0},
      .endpoint_xyz_valid = true,
      .attitude_compensation_applied = true,
      .applied_roll_rad = 0.1,
      .applied_pitch_rad = -0.2,
      .applied_tilt_rad = 0.3,
  };
  const AcceptedObstacleMemoryHit hit{
      .beam = observation,
      .known_static =
          KnownStaticClassificationSnapshot{
              .classifier_applied = true,
              .classification = KnownStaticLidarHitClassification::kUnexpected,
              .volume_matched = true,
              .confident_face_interior = true,
              .part_kind_valid = true,
              .part_kind = KnownPassageSolidPartKind::kUpper,
              .structure_id = "connector",
              .opening_id = "opening",
              .part_id = "upper_mass",
              .expected_range_m = 9.0,
              .range_delta_m = -2.5,
          },
      .ingestion_decision =
          LidarIngestionDecisionSnapshot{
              .action = LidarIngestionAction::kIntegrateFreeAndHit,
              .reason = LidarIngestionReason::kObstacleBeforeExpectedSurface,
              .expected_surface = LidarExpectedSurfaceKind::kKnownStatic,
              .expected_range_m = 9.0,
              .range_delta_m = -2.5,
          },
  };
  const LidarIngestionDecision decision{
      .action = LidarIngestionAction::kIntegrateFreeAndHit,
      .reason = LidarIngestionReason::kObstacleBeforeExpectedSurface,
      .expected_surface = LidarExpectedSurfaceKind::kKnownStatic,
      .ground_provider = LidarExpectedSurfaceProviderStatus::kReady,
      .known_static_provider = LidarExpectedSurfaceProviderStatus::kReady,
      .expected_range_m = 9.0,
      .range_delta_m = -2.5,
      .ground_candidate_considered = true,
      .expected_ground_range_m = 14.0,
      .known_static_surface =
          KnownStaticExpectedSurface{
              .range_m = 9.0,
              .intersection_map_m = Point3{14.0, 6.0, 7.0},
              .part_kind = KnownPassageSolidPartKind::kUpper,
              .structure_id = "connector",
              .opening_id = "opening",
              .part_id = "upper_mass",
              .volume_center = Point2{15.0, 6.0},
              .volume_normal_xy = Point2{1.0, 0.0},
              .volume_lateral_xy = Point2{0.0, 1.0},
              .volume_depth_m = 2.0,
              .volume_width_m = 7.0,
              .volume_min_z_m = 8.5,
              .volume_max_z_m = 32.0,
              .confident_face_interior = true,
          },
  };
  return LidarMemoryHitDiagnosticRecord{
      .record_index = 1U,
      .transition =
          ObstacleMemoryOccupiedTransition{
              .score_before = 2,
              .score_after = 6,
              .provenance =
                  MemoryCellProvenance{
                      .cell = GridIndex{23, 24},
                      .occupancy_trigger = hit,
                      .last_hit = hit,
                      .min_endpoint_z_m = 6.5,
                      .max_endpoint_z_m = 8.0,
                      .accepted_hit_count = 3U,
                  },
              .trigger_decision = decision,
          },
      .context =
          LidarMemoryHitDiagnosticContext{
              .callback_stamp_ns = 123'600'000,
              .pose_sample_stamp_ns = 123'500'000,
              .pose_sample_stamp_valid = true,
              .pose_receive_stamp_ns = 123'550'000,
              .pose_receive_stamp_valid = true,
              .attitude_sample_stamp_ns = 123'400'000,
              .attitude_sample_stamp_valid = true,
              .attitude_receive_stamp_ns = 123'580'000,
              .attitude_receive_stamp_valid = true,
              .vehicle_pose = LidarProjectionPose{Point2{5.0, 6.0}, 7.0, 0.0, 0.1, -0.2,
                                                  true, true},
              .horizontal_velocity = Point2{4.0, 0.0},
              .horizontal_velocity_valid = true,
              .motion_compensation =
                  LidarPoseMotionCompensationResult{
                      .position = Point2{5.1, 6.0},
                      .applied_shift = Point2{0.1, 0.0},
                      .pose_lag_s = 0.01,
                      .latency_s = 0.05,
                      .signed_time_offset_s = -0.04,
                      .applied = true,
                  },
              .scan_range_min_m = 0.2,
              .scan_range_max_m = 35.0,
              .scan_angle_min_rad = -1.0,
              .scan_angle_increment_rad = 0.01,
              .scan_time_increment_s = 0.001,
              .scan_duration_s = 0.72,
              .projection_config = LidarProjectionConfig{},
              .ground_config = GroundLidarRejectionConfig{},
              .known_static_closer_range_tolerance_m = 0.5,
              .known_static_farther_range_tolerance_m = 1.5,
              .known_static_endpoint_volume_tolerance_m = 0.75,
              .known_static_opening_boundary_tolerance_m = 0.15,
          },
  };
}

} // namespace

TEST(LidarMemoryHitDiagnostics, JsonIncludesRawBeamAndBothSurfaceCandidates) {
  std::ostringstream stream;
  writeLidarMemoryHitDiagnosticJson(stream, makeRecord());

  const std::string json = stream.str();
  EXPECT_NE(json.find("\"beam_index\":17"), std::string::npos);
  EXPECT_NE(json.find("\"ray_origin_map_m\":{\"x\":5"), std::string::npos);
  EXPECT_NE(json.find("\"endpoint_map_m\":{\"x\":11.5"), std::string::npos);
  EXPECT_NE(json.find("\"ground_range_m\":14"), std::string::npos);
  EXPECT_NE(json.find("\"intersection_map_m\":{\"x\":14"), std::string::npos);
  EXPECT_NE(json.find("\"part\":\"upper\""), std::string::npos);
  EXPECT_NE(json.find("\"selected_surface\":\"known_static\""), std::string::npos);
  EXPECT_NE(json.find("\"known_static_opening_boundary_tolerance_m\":0.15"),
            std::string::npos);
  EXPECT_NE(json.find("\"min_endpoint_z_m\":6.5"), std::string::npos);
}

TEST(LidarMemoryHitDiagnostics, WriterHonorsRecordLimit) {
  const std::filesystem::path path =
      std::filesystem::temp_directory_path() / "drone_city_nav_memory_hit_dump.jsonl";
  LidarMemoryHitDumpWriter writer;
  ASSERT_EQ(writer.open(LidarMemoryHitDumpConfig{true, path, 1U}),
            LidarMemoryHitDumpOpenStatus::kReady);

  EXPECT_EQ(writer.write(makeRecord()).status, LidarMemoryHitDumpWriteStatus::kWritten);
  const LidarMemoryHitDumpWriteResult limited = writer.write(makeRecord());
  EXPECT_EQ(limited.status, LidarMemoryHitDumpWriteStatus::kLimitReached);
  EXPECT_TRUE(limited.first_limit_reached);

  std::ifstream input{path};
  ASSERT_TRUE(input.is_open());
  const std::string content{std::istreambuf_iterator<char>{input},
                            std::istreambuf_iterator<char>{}};
  EXPECT_NE(content.find("\"record_index\":1"), std::string::npos);

  std::error_code ignored;
  std::filesystem::remove(path, ignored);
}

TEST(LidarMemoryHitDiagnostics, TimestampAndScanDurationHelpersKeepDiagnosticInputs) {
  EXPECT_EQ(px4TimestampNanoseconds(123U), 123'000);
  EXPECT_FALSE(px4TimestampNanoseconds(0U).has_value());
  EXPECT_DOUBLE_EQ(lidarScanDurationSeconds(0.72, 0.001, 721U), 0.72);
  EXPECT_DOUBLE_EQ(lidarScanDurationSeconds(0.0, 0.001, 721U), 0.72);
}

} // namespace drone_city_nav
