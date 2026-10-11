// replay_lio DIR HEADING_NED_RAD [key=value config overrides]: runs the lidar-inertial estimator over
// extract_bag.py output (DIR/scans.bin, DIR/imu.csv), merging IMU and scans by stamp (IMU delivered up to
// the scan stamp), and writes DIR/replay.csv (or out=PATH): stamp_ns,x,y,z,healthy,information,degenerate,
// bias_x,bias_y,bias_z,vx,vy,vz, plus PATH.ms with the registration time p50 p95 p99 max and the count
// over 100 ms. Keys: lag gate keyframes kftrans iterations corr recovery cell info floor acc range points voxel;
// hole_from and hole_to (seconds from the first scan) drop the scans of that window, so that the scan after it
// must register from a prior the IMU alone carried: the recovery's case.
#include "drone_city_nav/lidar_inertial_odometry.hpp"
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
  std::string out_path;
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
    else { std::fprintf(stderr, "unknown %s\n", k.c_str()); return 2; }
  }
  std::vector<Imu> imu;
  { std::FILE* f = std::fopen((dir + "/imu.csv").c_str(), "r"); Imu s;
    while (std::fscanf(f, "%lld,%lf,%lf,%lf,%lf,%lf,%lf", &s.t, &s.g[0], &s.g[1], &s.g[2], &s.a[0], &s.a[1], &s.a[2]) == 7) imu.push_back(s);
    std::fclose(f); }
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
    while (next_imu < imu.size() && imu[next_imu].t <= stamp - lag_ns) {
      const Imu& s = imu[next_imu++];
      if (!odometry.initialized()) odometry.initialize(s.t, Eigen::Vector3d::Zero(), heading);
      odometry.addImu(LidarInertialImuSample{.stamp_ns = s.t, .gyro_radps = {s.g[0], s.g[1], s.g[2]}, .accelerometer_mps2 = {s.a[0], s.a[1], s.a[2]}});
    }
    if (!odometry.initialized()) continue;
    if (first_stamp < 0) first_stamp = stamp;
    const double since_first_s = static_cast<double>(stamp - first_stamp) * 1e-9;
    if (since_first_s >= hole_from_s && since_first_s < hole_to_s) continue;
    std::vector<Eigen::Vector3d> points; points.reserve(n);
    for (unsigned i = 0; i < n; ++i) points.emplace_back(buffer[3 * i], buffer[3 * i + 1], buffer[3 * i + 2]);
    const auto started = std::chrono::steady_clock::now();
    const LidarInertialEstimate e = odometry.addScan(stamp, points);
    durations.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count());
    std::fprintf(out, "%lld,%.4f,%.4f,%.4f,%d,%.4f,%zu,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n", stamp, e.position_ned_m.x(), e.position_ned_m.y(), e.position_ned_m.z(), e.healthy ? 1 : 0, e.information_per_point, e.degenerate_axes, 0.0, 0.0, 0.0, e.velocity_ned_mps.x(), e.velocity_ned_mps.y(), e.velocity_ned_mps.z());
  }
  std::fclose(out); std::fclose(scans);
  std::sort(durations.begin(), durations.end());
  if (!durations.empty()) { std::FILE* t = std::fopen((out_path + ".ms").c_str(), "w"); std::fprintf(t, "%.1f %.1f %.1f %.1f over100=%zu of %zu\n", durations[durations.size() / 2], durations[durations.size() * 95 / 100], durations[durations.size() * 99 / 100], durations.back(), static_cast<std::size_t>(std::count_if(durations.begin(), durations.end(), [](double d){return d>100.0;})), durations.size()); std::fclose(t); }
  return 0;
}
