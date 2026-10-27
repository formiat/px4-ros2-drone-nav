// openvins_euroc.cpp SEQUENCE_DIR CONFIG_YAML OUT_CSV: OpenVINS (rpng/open_vins, ROS-free, the tool image of
// tools/tools_image.sh) over an EuRoC MAV sequence in the ASL format (mav0/imu0, cam0, cam1,
// state_groundtruth_estimate0), as the reference beside our visual-inertial estimator (roadmap item 25): the
// stereo-inertial MSCKF with the EuRoC configuration of the OpenVINS repository (config/euroc_mav), fed the IMU
// and both images in order (every image after the IMU that follows it), its estimate written after every image
// in the replay CSV of tools/vio/replay_vio.cpp (stamp_s from the first IMU sample, the IMU's position and
// orientation in OpenVINS's gravity-aligned global frame, the reference of that stamp interpolated from the
// ground truth, the rest of the columns the feature counts the filter reports and noughts), so that
// tools/vio/euroc_report.py reads it as it reads ours. Nothing here enters the flight; the tool image only.
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/sensor_data.h"
#include <opencv2/imgcodecs.hpp>
#include <Eigen/Dense>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace {
struct Truth { double t; Eigen::Vector3d p; Eigen::Quaterniond q; };

std::vector<std::pair<long long, std::string>> readFrames(const std::string& csv) {
  std::vector<std::pair<long long, std::string>> frames;
  std::ifstream in(csv); std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') continue;
    while (!line.empty() && (line.back() == '\r' || line.back() == ' ')) line.pop_back();  // the CSVs end their lines CRLF
    const auto comma = line.find(',');
    frames.emplace_back(std::stoll(line.substr(0, comma)), line.substr(comma + 1));
  }
  return frames;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) { std::fprintf(stderr, "usage: openvins_euroc SEQUENCE_DIR CONFIG_YAML OUT_CSV\n"); return 2; }
  const std::string dir = argv[1], config = argv[2], out_path = argv[3];
  auto parser = std::make_shared<ov_core::YamlParser>(config);
  ov_msckf::VioManagerOptions params;
  params.print_and_load(parser);
  params.num_opencv_threads = 0;
  if (!parser->successful()) { std::fprintf(stderr, "the configuration did not parse\n"); return 1; }
  auto sys = std::make_shared<ov_msckf::VioManager>(params);
  // The IMU (ns, wx, wy, wz, ax, ay, az) and the reference (ns, p, q(w,x,y,z), ...).
  std::vector<ov_core::ImuData> imu;
  { std::ifstream in(dir + "/mav0/imu0/data.csv"); std::string line;
    while (std::getline(in, line)) { if (line.empty() || line[0] == '#') continue;
      std::stringstream s(line); std::string f; std::vector<double> v;
      while (std::getline(s, f, ',')) v.push_back(std::stod(f));
      ov_core::ImuData m; m.timestamp = v[0] * 1e-9; m.wm << v[1], v[2], v[3]; m.am << v[4], v[5], v[6]; imu.push_back(m); } }
  std::vector<Truth> truth;
  { std::ifstream in(dir + "/mav0/state_groundtruth_estimate0/data.csv"); std::string line;
    while (std::getline(in, line)) { if (line.empty() || line[0] == '#') continue;
      std::stringstream s(line); std::string f; std::vector<double> v;
      while (std::getline(s, f, ',')) v.push_back(std::stod(f));
      truth.push_back({v[0] * 1e-9, {v[1], v[2], v[3]}, Eigen::Quaterniond(v[4], v[5], v[6], v[7])}); } }
  if (imu.empty() || truth.empty()) { std::fprintf(stderr, "no IMU or no reference\n"); return 1; }
  const auto left = readFrames(dir + "/mav0/cam0/data.csv");
  std::map<long long, std::string> right;
  for (const auto& f : readFrames(dir + "/mav0/cam1/data.csv")) right[f.first] = f.second;
  const double t0 = imu.front().timestamp;
  FILE* out = std::fopen(out_path.c_str(), "w");
  std::fprintf(out, "stamp_s,px,py,pz,qw,qx,qy,qz,vx,vy,vz,tx,ty,tz,tqw,tqx,tqy,tqz,tracked,candidates,used,gated,untriangulated,rms,ms_track,ms_filter,bgx,bgy,bgz,bax,bay,baz\n");
  std::size_t next_imu = 0, next_truth = 0, written = 0, frames = 0;
  for (const auto& [stamp_ns, name] : left) {
    const auto partner = right.find(stamp_ns);
    if (partner == right.end()) continue;
    const double t = stamp_ns * 1e-9;
    // The IMU up to and past the image, as a driver delivers it.
    while (next_imu < imu.size() && imu[next_imu].timestamp <= t + 0.001) sys->feed_measurement_imu(imu[next_imu++]);
    ov_core::CameraData cam; cam.timestamp = t; cam.sensor_ids = {0, 1};
    cam.images = {cv::imread(dir + "/mav0/cam0/data/" + name, cv::IMREAD_GRAYSCALE),
                  cv::imread(dir + "/mav0/cam1/data/" + partner->second, cv::IMREAD_GRAYSCALE)};
    if (cam.images[0].empty() || cam.images[1].empty()) continue;
    cam.masks = {cv::Mat::zeros(cam.images[0].size(), CV_8UC1), cv::Mat::zeros(cam.images[1].size(), CV_8UC1)};
    sys->feed_measurement_camera(cam);
    ++frames;
    if (!sys->initialized()) continue;
    // The reference at this stamp, interpolated between its neighbours.
    while (next_truth + 1 < truth.size() && truth[next_truth + 1].t < t) ++next_truth;
    if (next_truth + 1 >= truth.size() || truth[next_truth].t > t) continue;
    const Truth& a = truth[next_truth]; const Truth& b = truth[next_truth + 1];
    const double s = (t - a.t) / std::max(b.t - a.t, 1e-9);
    const Eigen::Vector3d tp = a.p + s * (b.p - a.p);
    const Eigen::Quaterniond tq = a.q.slerp(s, b.q);
    const auto state = sys->get_state();
    const Eigen::Vector3d p = state->_imu->pos();
    const Eigen::Matrix<double, 4, 1> q = state->_imu->quat();  // JPL, (x, y, z, w), q_GtoI
    const Eigen::Vector3d v = state->_imu->vel();
    const Eigen::Vector3d bg = state->_imu->bias_g(), ba = state->_imu->bias_a();
    // The body-to-global orientation as a Hamilton quaternion: the conjugate of q_GtoI.
    std::fprintf(out, "%.6f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%zu,%zu,%zu,%zu,%zu,%.3f,%.2f,%.2f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n",
                 t - t0, p.x(), p.y(), p.z(), q(3), -q(0), -q(1), -q(2), v.x(), v.y(), v.z(), tp.x(), tp.y(), tp.z(),
                 tq.w(), tq.x(), tq.y(), tq.z(), sys->get_features_SLAM().size() + sys->get_good_features_MSCKF().size(),
                 sys->get_good_features_MSCKF().size(), sys->get_good_features_MSCKF().size(), std::size_t{0}, std::size_t{0},
                 0.0, 0.0, 0.0, bg.x(), bg.y(), bg.z(), ba.x(), ba.y(), ba.z());
    ++written;
  }
  std::fclose(out);
  std::fprintf(stderr, "OPENVINS frames=%zu written=%zu initialized_at=%.2f\n", frames, written,
               sys->initialized() ? sys->initialized_time() - t0 : -1.0);
  return 0;
}
