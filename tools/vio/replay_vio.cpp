// replay_vio DIR [key=value ...]: runs the stereo feature tracker and the visual-inertial filter over a record of record_vio.py.
//   imu=px4|gz (default px4)   offset_ms=F  added to every frame stamp   every=N  use every N-th frame   start_s=T first frame not before T
//   prior=0|1 gyro rotation prior for the tracker (default 1)   scale=1|2 image downscale   features=N   clones=N   noise_px=F
// The declared initial pose is the true pose at the first used frame (the vehicle must be at rest there).
// Output DIR/replay_<tag>.csv: stamp_s,px,py,pz,qw,qx,qy,qz,vx,vy,vz,tx,ty,tz,tqw,tqx,tqy,tqz,tracked,candidates,used,gated,untriangulated,rms,ms_track,ms_filter
#include "drone_city_nav/visual_inertial_odometry.hpp"
#include "stereo_feature_tracker.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>
using namespace drone_city_nav;
struct Imu { double t; Eigen::Vector3d g, a; };
struct Truth { double t; Eigen::Vector3d p; Eigen::Quaterniond q; };
struct Frame { double t; int w, h; long long left_offset, right_offset; };
static std::vector<std::vector<std::string>> csv(const std::string& path) {
  std::vector<std::vector<std::string>> rows; std::ifstream f(path); std::string line; std::getline(f, line);
  while (std::getline(f, line)) { std::vector<std::string> c; std::stringstream s(line); std::string x; while (std::getline(s, x, ',')) c.push_back(x); if (!c.empty()) rows.push_back(c); }
  return rows;
}
int main(int argc, char** argv) {
  const std::string dir = argv[1];
  std::map<std::string, std::string> opt{{"imu", "px4"}, {"offset_ms", "0"}, {"every", "1"}, {"start_s", "0"}, {"prior", "1"}, {"scale", "1"}, {"features", "200"}, {"clones", "12"}, {"noise_px", "1.0"}, {"tag", "run"}, {"end_s", "1e9"}, {"dark_from_s", "-1"}, {"dark_to_s", "-1"}, {"baro", "0"}, {"ramp_s", "0"}};
  for (int i = 2; i < argc; ++i) { std::string kv = argv[i]; auto eq = kv.find('='); opt[kv.substr(0, eq)] = kv.substr(eq + 1); }
  const double offset_s = std::atof(opt["offset_ms"].c_str()) * 1e-3; const int every = std::atoi(opt["every"].c_str()); const int scale = std::atoi(opt["scale"].c_str());
  // ENU/FLU truth -> NED/FRD
  Eigen::Matrix3d enu_to_ned; enu_to_ned << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  const Eigen::Matrix3d frd_to_flu = Eigen::Vector3d{1, -1, -1}.asDiagonal();
  std::vector<Imu> imu;
  for (auto& r : csv(dir + (opt["imu"] == "gz" ? "/imu_gz.csv" : "/imu_px4.csv"))) {
    Imu s{std::atof(r[0].c_str()), {std::atof(r[1].c_str()), std::atof(r[2].c_str()), std::atof(r[3].c_str())}, {std::atof(r[4].c_str()), std::atof(r[5].c_str()), std::atof(r[6].c_str())}};
    if (opt["imu"] == "gz") { s.g = frd_to_flu * s.g; s.a = frd_to_flu * s.a; }
    imu.push_back(s);
  }
  std::vector<Truth> truth;
  for (auto& r : csv(dir + "/truth.csv")) {
    const Eigen::Quaterniond q_enu_flu{std::atof(r[4].c_str()), std::atof(r[5].c_str()), std::atof(r[6].c_str()), std::atof(r[7].c_str())};
    truth.push_back({std::atof(r[0].c_str()), enu_to_ned * Eigen::Vector3d{std::atof(r[1].c_str()), std::atof(r[2].c_str()), std::atof(r[3].c_str())},
                     Eigen::Quaterniond{enu_to_ned * q_enu_flu.toRotationMatrix() * frd_to_flu}});
  }
  std::map<std::string, Frame> frames;  // by stamp text: both sides share a stamp
  for (auto& r : csv(dir + "/frames.csv")) { Frame& f = frames[r[1]]; f.t = std::atof(r[1].c_str()); f.w = std::atoi(r[3].c_str()); f.h = std::atoi(r[4].c_str()); (r[0] == "left" ? f.left_offset : f.right_offset) = std::atoll(r[5].c_str()) + 1; }
  std::vector<Frame> ordered; for (auto& [k, f] : frames) if (f.left_offset > 0 && f.right_offset > 0) ordered.push_back(f);
  std::sort(ordered.begin(), ordered.end(), [](const Frame& a, const Frame& b) { return a.t < b.t; });
  const auto truthAt = [&](double t) { std::size_t i = 0; while (i + 1 < truth.size() && truth[i + 1].t <= t) ++i; if (i + 1 >= truth.size()) return truth.back(); const double u = (t - truth[i].t) / (truth[i + 1].t - truth[i].t); return Truth{t, truth[i].p + u * (truth[i + 1].p - truth[i].p), truth[i].q.slerp(u, truth[i + 1].q)}; };

  Eigen::Matrix3d camera_to_body; camera_to_body << 0, 0, 1, 1, 0, 0, 0, 1, 0;
  VisualInertialOdometryConfig config;
  config.left_camera = {Eigen::Quaterniond{camera_to_body}, {0.32, -0.10, -0.02}};
  config.right_camera = {Eigen::Quaterniond{camera_to_body}, {0.32, 0.10, -0.02}};
  config.maximum_clones = static_cast<std::size_t>(std::atoi(opt["clones"].c_str()));
  StereoFeatureTrackerConfig tracker_config;
  tracker_config.image_width = static_cast<std::size_t>(ordered.front().w / scale); tracker_config.image_height = static_cast<std::size_t>(ordered.front().h / scale);
  tracker_config.maximum_features = static_cast<std::size_t>(std::atoi(opt["features"].c_str()));
  if (scale > 1) { tracker_config.minimum_distance_px /= scale; tracker_config.maximum_disparity_px /= scale; tracker_config.minimum_disparity_px /= scale; }
  for (auto& [k, v] : opt) {
    if (k == "gyro_noise") config.gyro_noise_radps_sqrt_hz = std::atof(v.c_str());
    if (k == "acc_noise") config.accelerometer_noise_mps2_sqrt_hz = std::atof(v.c_str());
    if (k == "gate") config.gate_normal_quantile = std::atof(v.c_str());
    if (k == "drag") config.rotor_drag_1ps = std::atof(v.c_str());
    if (k == "per_update") config.maximum_features_per_update = static_cast<std::size_t>(std::atoi(v.c_str()));
    if (k == "bg_walk") config.gyro_bias_walk_radps2_sqrt_hz = std::atof(v.c_str());
    if (k == "bg_sigma") config.minimum_gyro_bias_sigma_radps = std::atof(v.c_str());
    if (k == "ba_walk") config.accelerometer_bias_walk_mps3_sqrt_hz = std::atof(v.c_str());
    if (k == "baseline") { config.left_camera.position_body_m.y() = -0.5 * std::atof(v.c_str()); config.right_camera.position_body_m.y() = 0.5 * std::atof(v.c_str()); }
    if (k == "maxdepth") config.maximum_depth_m = std::atof(v.c_str());
    if (k == "mindisp") tracker_config.minimum_disparity_px = std::atof(v.c_str());
    if (k == "fov") tracker_config.horizontal_fov_rad = std::atof(v.c_str());
    if (k == "minframes") config.minimum_track_frames = static_cast<std::size_t>(std::atoi(v.c_str()));
  }
  const double focal = 0.5 * tracker_config.image_width / std::tan(0.5 * tracker_config.horizontal_fov_rad);
  config.observation_noise = std::atof(opt["noise_px"].c_str()) / focal;
  VisualInertialOdometry odometry{config};
  StereoFeatureTracker tracker{tracker_config};
  std::FILE* left = std::fopen((dir + "/left.bin").c_str(), "rb"); std::FILE* right = std::fopen((dir + "/right.bin").c_str(), "rb");
  std::FILE* out = std::fopen((dir + "/replay_" + opt["tag"] + ".csv").c_str(), "w");
  std::fprintf(out, "stamp_s,px,py,pz,qw,qx,qy,qz,vx,vy,vz,tx,ty,tz,tqw,tqx,tqy,tqz,tracked,candidates,used,gated,untriangulated,rms,ms_track,ms_filter,bgx,bgy,bgz,bax,bay,baz\n");
  std::size_t next_imu = 0; double previous_stamp = -1.0; int index = 0; double baro_h = -truth.front().p.z();
  cv::Mat l_full, r_full, l, r;
  for (const Frame& f : ordered) {
    if (f.t < std::atof(opt["start_s"].c_str()) || f.t > std::atof(opt["end_s"].c_str()) || (index++ % every) != 0) continue;
    const double stamp = f.t + offset_s;
    l_full.create(f.h, f.w, CV_8UC1); r_full.create(f.h, f.w, CV_8UC1);
    std::fseek(left, f.left_offset - 1, SEEK_SET); std::fseek(right, f.right_offset - 1, SEEK_SET);
    if (std::fread(l_full.data, 1, static_cast<std::size_t>(f.w) * f.h, left) != static_cast<std::size_t>(f.w) * f.h || std::fread(r_full.data, 1, static_cast<std::size_t>(f.w) * f.h, right) != static_cast<std::size_t>(f.w) * f.h) break;
    if (scale > 1) { cv::resize(l_full, l, {}, 1.0 / scale, 1.0 / scale, cv::INTER_AREA); cv::resize(r_full, r, {}, 1.0 / scale, 1.0 / scale, cv::INTER_AREA); } else { l = l_full; r = r_full; }
    if (f.t >= std::atof(opt["dark_from_s"].c_str()) && f.t < std::atof(opt["dark_to_s"].c_str())) { l = cv::Mat::zeros(l.size(), l.type()); r = cv::Mat::zeros(r.size(), r.type()); }
    { const double from = std::atof(opt["dark_from_s"].c_str()), ramp = std::atof(opt["ramp_s"].c_str());
      if (ramp > 0.0 && f.t >= from - ramp && f.t < from) {
        const double share = std::max(0.02, (from - f.t) / ramp), gain = std::min(8.0, 1.0 / share);
        static cv::RNG rng{5};
        for (cv::Mat* m : {&l, &r}) {
          cv::Mat v; m->convertTo(v, CV_32F, share * gain);
          cv::Mat n(v.size(), CV_32F); rng.fill(n, cv::RNG::NORMAL, 0.0, 2.55 * (gain - 1.0));
          v += n; v.convertTo(*m, CV_8U);
        }
      } }
    std::optional<Eigen::Matrix3d> prior;
    Eigen::Matrix3d turn = Eigen::Matrix3d::Identity();
    if (!odometry.initialized()) {
      // samples at rest before the declared pose
      while (next_imu < imu.size() && imu[next_imu].t <= stamp) { odometry.addImu({static_cast<std::int64_t>(imu[next_imu].t * 1e9), imu[next_imu].g, imu[next_imu].a}); ++next_imu; }
      const Truth t0 = truthAt(f.t); const Eigen::Matrix3d R = t0.q.toRotationMatrix();
      odometry.initialize(static_cast<std::int64_t>(stamp * 1e9), t0.p, std::atan2(R(1, 0), R(0, 0)));
    } else {
      while (next_imu < imu.size() && imu[next_imu].t <= stamp) {
        const Imu& s = imu[next_imu]; const double from = std::max(previous_stamp, next_imu > 0 ? imu[next_imu - 1].t : s.t);
        const Eigen::Vector3d w = s.g * (s.t - from); const double angle = w.norm();
        if (angle > 1e-12) turn = turn * Eigen::AngleAxisd{angle, w / angle}.toRotationMatrix();
        odometry.addImu({static_cast<std::int64_t>(s.t * 1e9), s.g, s.a}); ++next_imu;
        if (opt["baro"] == "1") { static std::mt19937 g{3}; std::normal_distribution<double> n{0.0, 0.25}; baro_h = 0.96 * baro_h + 0.04 * (-truthAt(s.t).p.z() + n(g)); odometry.addBarometricHeight(baro_h); }
      }
      if (opt["prior"] == "1") prior = camera_to_body.transpose() * turn.transpose() * camera_to_body;
    }
    const auto t0 = std::chrono::steady_clock::now();
    const std::vector<StereoFeatureObservation> observations = tracker.track(l, r, prior);
    const auto t1 = std::chrono::steady_clock::now();
    const VisualInertialEstimate e = odometry.addFrame(static_cast<std::int64_t>(stamp * 1e9), observations);
    const auto t2 = std::chrono::steady_clock::now();
    const Truth t = truthAt(f.t);
    std::fprintf(out, "%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%zu,%zu,%zu,%zu,%zu,%.3f,%.2f,%.2f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n", f.t,
                 e.position_ned_m.x(), e.position_ned_m.y(), e.position_ned_m.z(), e.body_to_ned.w(), e.body_to_ned.x(), e.body_to_ned.y(), e.body_to_ned.z(),
                 e.velocity_ned_mps.x(), e.velocity_ned_mps.y(), e.velocity_ned_mps.z(), t.p.x(), t.p.y(), t.p.z(), t.q.w(), t.q.x(), t.q.y(), t.q.z(),
                 observations.size(), e.candidate_features, e.used_features, e.gated_features, e.untriangulated_features, e.residual_rms_sigma,
                 std::chrono::duration<double, std::milli>(t1 - t0).count(), std::chrono::duration<double, std::milli>(t2 - t1).count(),
                 e.gyro_bias_radps.x(), e.gyro_bias_radps.y(), e.gyro_bias_radps.z(), e.accelerometer_bias_mps2.x(), e.accelerometer_bias_mps2.y(), e.accelerometer_bias_mps2.z());
    previous_stamp = stamp;
  }
  std::fclose(out);
  return 0;
}
