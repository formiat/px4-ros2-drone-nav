#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace drone_city_nav {

TEST(Px4OffboardConfig, DefaultYamlExposesTimeAwareRacingLineParameters) {
  const std::string config_path =
      std::string{DRONE_CITY_NAV_SOURCE_DIR} + "/config/urban_mvp.yaml";
  std::ifstream stream{config_path};
  ASSERT_TRUE(stream.is_open()) << config_path;

  std::ostringstream buffer;
  buffer << stream.rdbuf();
  const std::string yaml = buffer.str();

  EXPECT_NE(yaml.find("racing_line_weight_time:"), std::string::npos);
  EXPECT_NE(yaml.find("racing_line_weight_length: 0.02"), std::string::npos);
  EXPECT_NE(yaml.find("racing_line_weight_edge_margin:"), std::string::npos);
  EXPECT_NE(yaml.find("racing_line_desired_edge_margin_m:"), std::string::npos);
  EXPECT_NE(yaml.find("racing_line_regularization_iterations:"), std::string::npos);
  EXPECT_NE(yaml.find("racing_line_regularization_max_time_regression_s:"),
            std::string::npos);
  EXPECT_NE(yaml.find("speed_profile_decel_mps2: 2.0"), std::string::npos);
  EXPECT_NE(yaml.find("cross_track_gain: 0.5"), std::string::npos);
  EXPECT_NE(yaml.find("max_cross_track_correction_angle_deg: 45.0"), std::string::npos);
}

} // namespace drone_city_nav
