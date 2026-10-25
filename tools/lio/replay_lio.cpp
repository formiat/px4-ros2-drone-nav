// replay_lio DIR HEADING_NED_RAD [key=value config overrides]: runs the lidar-inertial estimator over
// extract_bag.py output (DIR/scans.bin, DIR/imu.csv), merging IMU and scans by stamp (IMU delivered up to
// the scan stamp), and writes DIR/replay.csv (or out=PATH): stamp_ns,x,y,z,healthy,information,degenerate,
// bias_x,bias_y,bias_z,vx,vy,vz,roll,pitch (rad, the estimate's tilt), plus PATH.ms with the registration time p50 p95 p99 max and the count
// over 100 ms. Keys: lag gate keyframes kftrans iterations corr recovery cell info floor acc range points voxel tiltshare
// (the registration's tilt prior share), truth=PATH (the flight's gz_pose.csv as the outside level), attitude=PATH (the
// record's attitude.csv, the autopilot's own attitude as the level), baro=PATH and heightshare=S (the record's baro.csv as
// the outside height, and the share of the height prior) and misalign=DEG (the
// IMU turned about the body's y axis, a mount error that seeds a tilted map);
// hole_from and hole_to (seconds from the first scan) drop the scans of that window, so that the scan after it
// must register from a prior the IMU alone carried: the recovery's case.
#include "drone_city_nav/lidar_inertial_odometry.hpp"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
using namespace drone_city_nav;
struct Imu { long long t; double g[3], a[3]; };
int main(int argc, char** argv) {
  std::string dir = argv[1];
  double heading = std::atof(argv[2]);
  LidarInertialOdometryConfig config;
  long long lag_ns = 0;
  double hole_from_s = -1.0, hole_to_s = -1.0;
  std::string out_path, truth_path, attitude_path, baro_path;
  double misalign_deg = 0.0;  // the IMU turned about the body's y axis by this much, a mount error, to seed a tilted map
  for (int i = 3; i < argc; ++i) {
    std::string kv = argv[i]; auto eq = kv.find('='); std::string k = kv.substr(0, eq); double v = std::atof(kv.c_str() + eq + 1);
    if (k == "out") { out_path = kv.substr(eq + 1); continue; }
    if (k == "lag") lag_ns = static_cast<long long>(v * 1e6);
    else if (k == "hole_from") hole_from_s = v;
    else if (k == "hole_to") hole_to_s = v;
    else if (k == "gate") config.innovation_gate_sigma = v;
    else if (k == "keyframes") config.maximum_keyframes = static_cast<std::size_t>(v);
    else if (k == "kftrans") config.keyframe_translation_m = v;
    else if (k == "iterations") config.maximum_iterations = static_cast<std::size_t>(v);
    else if (k == "corr") config.maximum_correspondence_m = v;
    else if (k == "recovery") config.recovery_correspondence_factor = v;
    else if (k == "cell") config.maximum_points_per_cell = static_cast<std::size_t>(v);
    else if (k == "info") config.minimum_information_per_point = v;
    else if (k == "floor") config.minimum_position_variance_m2 = v;
    else if (k == "acc") config.acceleration_noise_mps2 = v;
    else if (k == "range") config.maximum_range_m = v;
    else if (k == "points") config.maximum_scan_points = static_cast<std::size_t>(v);
    else if (k == "voxel") config.scan_voxel_m = v;
    else if (k == "tiltshare") config.tilt_prior_share = v;
    else if (k == "misalign") misalign_deg = v;
    else if (k == "truth") { truth_path = kv.substr(eq + 1); continue; }
    else if (k == "attitude") { attitude_path = kv.substr(eq + 1); continue; }
    else if (k == "baro") { baro_path = kv.substr(eq + 1); continue; }
    else if (k == "heightshare") config.height_prior_share = v;
    else { std::fprintf(stderr, "unknown %s\n", k.c_str()); return 2; }
  }
  std::vector<Imu> imu;
  { std::FILE* f = std::fopen((dir + "/imu.csv").c_str(), "r"); Imu s;
    while (std::fscanf(f, "%lld,%lf,%lf,%lf,%lf,%lf,%lf", &s.t, &s.g[0], &s.g[1], &s.g[2], &s.a[0], &s.a[1], &s.a[2]) == 7) imu.push_back(s);
    std::fclose(f); }
  // The truth's attitude (gz_pose.csv: t, x, y, z, qw, qx, qy, qz, body FLU to the ENU map) as the outside level,
  // turned to body FRD to NED; fed before each scan at the scan's stamp, the ideal level the autopilot's stands in for.
  struct Truth { double t; Eigen::Quaterniond q; };
  std::vector<Truth> truth;
  if (!truth_path.empty()) {
    std::FILE* f = std::fopen(truth_path.c_str(), "r"); double t, x, y, z, qw, qx, qy, qz;
    Eigen::Matrix3d enu_to_ned; enu_to_ned << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    const Eigen::Matrix3d frd_to_flu = Eigen::Vector3d{1.0, -1.0, -1.0}.asDiagonal();
    while (f && std::fscanf(f, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf%*[^\n]", &t, &x, &y, &z, &qw, &qx, &qy, &qz) == 8) {
      const Eigen::Matrix3d r = enu_to_ned * Eigen::Quaterniond{qw, qx, qy, qz}.toRotationMatrix() * frd_to_flu;
      truth.push_back({t, Eigen::Quaterniond{r}});
    }
    if (f) std::fclose(f);
    std::fprintf(stderr, "truth attitude samples %zu\n", truth.size());
  }
  // The autopilot's attitude as recorded (attitude.csv of extract_bag.py: stamp_ns, qw, qx, qy, qz, body FRD to NED),
  // the level the flight itself had; the truth and this are exclusive, the attitude read first.
  if (!attitude_path.empty()) {
    std::FILE* f = std::fopen(attitude_path.c_str(), "r"); long long ns; double qw, qx, qy, qz;
    truth.clear();
    while (f && std::fscanf(f, "%lld,%lf,%lf,%lf,%lf", &ns, &qw, &qx, &qy, &qz) == 5) truth.push_back({static_cast<double>(ns) * 1e-9, Eigen::Quaterniond{qw, qx, qy, qz}});
    if (f) std::fclose(f);
    std::fprintf(stderr, "autopilot attitude samples %zu\n", truth.size());
  }
  // The barometer as recorded (baro.csv of extract_bag.py: stamp_ns, pressure_pa), the height the flight itself had.
  struct Baro { long long t; double height_up_m; };
  std::vector<Baro> baro;
  if (!baro_path.empty()) {
    std::FILE* f = std::fopen(baro_path.c_str(), "r"); long long ns; double pa;
    while (f && std::fscanf(f, "%lld,%lf", &ns, &pa) == 2) baro.push_back({ns, -pa / 12.013});
    if (f) std::fclose(f);
    std::fprintf(stderr, "barometer samples %zu\n", baro.size());
  }
  std::size_t next_baro = 0;
  std::size_t next_truth = 0;
  std::FILE* scans = std::fopen((dir + "/scans.bin").c_str(), "rb");
  if (out_path.empty()) out_path = dir + "/replay.csv";
  std::FILE* out = std::fopen(out_path.c_str(), "w");
  LidarInertialOdometry odometry{config};
  std::size_t next_imu = 0;
  long long stamp; unsigned n;
  std::vector<float> buffer;
  std::vector<double> durations;
  long long first_stamp = -1;
  while (std::fread(&stamp, sizeof stamp, 1, scans) == 1 && std::fread(&n, sizeof n, 1, scans) == 1) {
    buffer.resize(3 * n);
    if (std::fread(buffer.data(), sizeof(float), 3 * n, scans) != 3 * n) break;
    while (next_baro < baro.size() && baro[next_baro].t <= stamp) { if (odometry.initialized()) odometry.observeHeight(baro[next_baro].t, baro[next_baro].height_up_m); ++next_baro; }
    while (next_imu < imu.size() && imu[next_imu].t <= stamp - lag_ns) {
      const Imu& s = imu[next_imu++];
      if (!odometry.initialized()) odometry.initialize(s.t, Eigen::Vector3d::Zero(), heading);
      const Eigen::Matrix3d mount = Eigen::AngleAxisd{misalign_deg * M_PI / 180.0, Eigen::Vector3d::UnitY()}.toRotationMatrix();
      odometry.addImu(LidarInertialImuSample{.stamp_ns = s.t, .gyro_radps = mount * Eigen::Vector3d{s.g[0], s.g[1], s.g[2]}, .accelerometer_mps2 = mount * Eigen::Vector3d{s.a[0], s.a[1], s.a[2]}});
    }
    if (!odometry.initialized()) continue;
    if (first_stamp < 0) first_stamp = stamp;
    const double since_first_s = static_cast<double>(stamp - first_stamp) * 1e-9;
    if (since_first_s >= hole_from_s && since_first_s < hole_to_s) continue;
    std::vector<Eigen::Vector3d> points; points.reserve(n);
    for (unsigned i = 0; i < n; ++i) points.emplace_back(buffer[3 * i], buffer[3 * i + 1], buffer[3 * i + 2]);
    while (next_truth < truth.size() && truth[next_truth].t * 1e9 <= static_cast<double>(stamp)) ++next_truth;
    if (next_truth > 0 && !truth.empty()) odometry.observeAttitude(stamp, truth[next_truth - 1].q);
    const auto started = std::chrono::steady_clock::now();
    const LidarInertialEstimate e = odometry.addScan(stamp, points);
    durations.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count());
    std::fprintf(out, "%lld,%.4f,%.4f,%.4f,%d,%.4f,%zu,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f\n", stamp, e.position_ned_m.x(), e.position_ned_m.y(), e.position_ned_m.z(), e.healthy ? 1 : 0, e.information_per_point, e.degenerate_axes, 0.0, 0.0, 0.0, e.velocity_ned_mps.x(), e.velocity_ned_mps.y(), e.velocity_ned_mps.z(), std::atan2(2.0 * (e.body_to_ned.w() * e.body_to_ned.x() + e.body_to_ned.y() * e.body_to_ned.z()), 1.0 - 2.0 * (e.body_to_ned.x() * e.body_to_ned.x() + e.body_to_ned.y() * e.body_to_ned.y())), std::asin(std::max(-1.0, std::min(1.0, 2.0 * (e.body_to_ned.w() * e.body_to_ned.y() - e.body_to_ned.z() * e.body_to_ned.x())))));
  }
  std::fclose(out); std::fclose(scans);
  std::sort(durations.begin(), durations.end());
  if (!durations.empty()) { std::FILE* t = std::fopen((out_path + ".ms").c_str(), "w"); std::fprintf(t, "%.1f %.1f %.1f %.1f over100=%zu of %zu\n", durations[durations.size() / 2], durations[durations.size() * 95 / 100], durations[durations.size() * 99 / 100], durations.back(), static_cast<std::size_t>(std::count_if(durations.begin(), durations.end(), [](double d){return d>100.0;})), durations.size()); std::fclose(t); }
  return 0;
}
