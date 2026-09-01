#pragma once

namespace drone_city_nav {

struct ProductionWorldBuildTelemetry3D {
  double build_ms{0.0};
  double esdf_x_pass_ms{0.0};
  double esdf_y_pass_ms{0.0};
  double esdf_z_pass_ms{0.0};
  double esdf_finalize_ms{0.0};
  double conversion_ms{0.0};
  double upload_ms{0.0};
};

} // namespace drone_city_nav
