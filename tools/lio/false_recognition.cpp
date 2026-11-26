// false_recognition DIR TRUTH_CSV [pairs=N] [apart_m=D] [same_m=D] [seed=N] [threads=N] [out=PATH]: how often the
// lidar-inertial estimator's own scan registration would take one place for another (roadmap item 41: whether the
// lidar set needs a loop closure, and what a closure by geometry would risk in tunnels that look alike).
//
// DIR is a record of tools/lio/extract_bag.py (scans.bin: the scans in the body frame, forward-right-down),
// TRUTH_CSV the flight's gz_pose.csv. For a pair of scans A and B a submap is built around A as the estimator
// builds its own (the scans before A a keyframe apart, 0.5 m, forty at most, thinned to one point a 0.4 m cell
// within 35 m, six points a cell; here at their true poses) and scan B is registered against it from A's position
// with B's own true attitude, the hypothesis "B was taken where A was" of a vehicle that knows its heading: the
// estimator's point-to-plane registration with the estimator's parameters (registerPointsToPlanes: correspondences
// within 1 m, ten iterations, the Huber width 0.2 m). The registration is accepted by the estimator's own health
// rule: converged, 0.3 of the scan (or 200 points) matched, the residual within 0.5 m; "every axis observed" is
// its information per point at 0.005 or more, under which the estimator trusts the IMU along the weak axis.
//
//   different places   B at least apart_m (10) from A by the truth: an accepted registration is a false recognition
//   the same place     B within same_m (1) of A and thirty seconds from it: an accepted one is a true recognition,
//                      and where it puts B against the truth is printed
//
// The health rule is no recognition's gate (it keeps the estimator alive on a floor's 200 points), so beside it the
// tool prints the pairs of different places whose registration is as good as a true revisit's: the matched
// fraction no less than the 5th percentile of the same-place pairs registered within 0.5 m of the truth, the
// residual no more than their 95th.
//
// pairs (1000) of each kind are drawn with the seed and run on threads (the cores); the log says when it started,
// how many pairs there are and, every hundred, how many are done. Output: a summary on stdout and, with out=, one
// line a pair (kind, the two stamps, the true distance, accepted, matched fraction, residual, information per
// point, the registered pose's distance from the truth). Built by tools/lio/build_false_recognition.sh.
#include "drone_city_nav/lidar_inertial_odometry.hpp"
#include "drone_city_nav/point_plane_map_3d.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <map>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
using namespace drone_city_nav;
struct Scan { long long stamp; std::vector<Eigen::Vector3d> points; Eigen::Vector3d position; Eigen::Quaterniond rotation; double path; };
struct Result { int kind; long long a, b; double apart; bool accepted; double matched, residual, information, error; };
// One point a cell: the cell's centroid, as the estimator thins a scan (coarser until it fits its point budget).
static std::vector<Eigen::Vector3d> thin(const std::vector<Eigen::Vector3d>& points, const LidarInertialOdometryConfig& config) {
  double voxel = config.scan_voxel_m;
  for (;;) {
    struct Cell { Eigen::Vector3d sum{Eigen::Vector3d::Zero()}; std::size_t count{0U}; };
    std::unordered_map<long long, Cell> cells;
    for (const Eigen::Vector3d& p : points) {
      const double range = p.norm();
      if (!p.allFinite() || range < config.minimum_range_m || range > config.maximum_range_m) continue;
      const long long x = static_cast<long long>(std::floor(p.x() / voxel)), y = static_cast<long long>(std::floor(p.y() / voxel)), z = static_cast<long long>(std::floor(p.z() / voxel));
      Cell& cell = cells[((x & 0x1FFFFF) << 42) | ((y & 0x1FFFFF) << 21) | (z & 0x1FFFFF)];
      cell.sum += p; ++cell.count;
    }
    if (config.maximum_scan_points == 0U || cells.size() <= config.maximum_scan_points) {
      std::vector<std::pair<long long, Eigen::Vector3d>> ordered;
      for (auto& [key, cell] : cells) ordered.emplace_back(key, cell.sum / static_cast<double>(cell.count));
      std::sort(ordered.begin(), ordered.end(), [](const auto& l, const auto& r) { return l.first < r.first; });
      std::vector<Eigen::Vector3d> out; out.reserve(ordered.size());
      for (auto& [key, point] : ordered) out.push_back(point);
      return out;
    }
    voxel *= 1.25;
  }
}
int main(int argc, char** argv) {
  if (argc < 3) { std::fprintf(stderr, "false_recognition DIR TRUTH_CSV [pairs=N] [apart_m=D] [same_m=D] [seed=N] [threads=N] [out=PATH]\n"); return 2; }
  const std::string dir = argv[1], truth_path = argv[2];
  std::map<std::string, std::string> opt{{"pairs", "1000"}, {"apart_m", "10"}, {"same_m", "1"}, {"seed", "1"}, {"threads", std::to_string(std::max(1U, std::thread::hardware_concurrency()))}, {"out", ""}};
  for (int i = 3; i < argc; ++i) { const std::string kv = argv[i]; const auto eq = kv.find('='); opt[kv.substr(0, eq)] = kv.substr(eq + 1); }
  const LidarInertialOdometryConfig config;
  // The truth (stamp_s, x, y, z, qw, qx, qy, qz: east-north-up, the body forward-left-up) as north-east-down, the body forward-right-down.
  struct Truth { double t; Eigen::Vector3d p; Eigen::Quaterniond q; };
  std::vector<Truth> truth;
  { std::FILE* f = std::fopen(truth_path.c_str(), "r"); double t, x, y, z, qw, qx, qy, qz;
    Eigen::Matrix3d enu_to_ned; enu_to_ned << 0, 1, 0, 1, 0, 0, 0, 0, -1;
    const Eigen::Matrix3d frd_to_flu = Eigen::Vector3d{1.0, -1.0, -1.0}.asDiagonal();
    while (f && std::fscanf(f, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf%*[^\n]", &t, &x, &y, &z, &qw, &qx, &qy, &qz) == 8)
      truth.push_back({t, enu_to_ned * Eigen::Vector3d{x, y, z}, Eigen::Quaterniond{enu_to_ned * Eigen::Quaterniond{qw, qx, qy, qz}.toRotationMatrix() * frd_to_flu}});
    if (f) std::fclose(f); }
  if (truth.empty()) { std::fprintf(stderr, "no truth in %s\n", truth_path.c_str()); return 1; }
  std::vector<Scan> scans;
  { std::FILE* f = std::fopen((dir + "/scans.bin").c_str(), "rb"); long long stamp; unsigned n; std::vector<float> buffer; std::size_t next = 0;
    while (f && std::fread(&stamp, sizeof stamp, 1, f) == 1 && std::fread(&n, sizeof n, 1, f) == 1) {
      buffer.resize(3 * n);
      if (std::fread(buffer.data(), sizeof(float), 3 * n, f) != 3 * n) break;
      const double t = static_cast<double>(stamp) * 1e-9;
      while (next + 1 < truth.size() && truth[next + 1].t <= t) ++next;
      if (t < truth.front().t || t > truth.back().t) continue;
      Scan scan{stamp, {}, truth[next].p, truth[next].q, 0.0};
      std::vector<Eigen::Vector3d> points; points.reserve(n);
      for (unsigned i = 0; i < n; ++i) points.emplace_back(buffer[3 * i], buffer[3 * i + 1], buffer[3 * i + 2]);
      scan.points = thin(points, config);
      scan.path = scans.empty() ? 0.0 : scans.back().path + (scan.position - scans.back().position).norm();
      scans.push_back(std::move(scan));
    }
    if (f) std::fclose(f); }
  if (scans.size() < 100U) { std::fprintf(stderr, "only %zu scans within the truth's span\n", scans.size()); return 1; }
  // The pairs: A with a submap's worth of path behind it; B elsewhere, or at the same place half a minute apart.
  const std::size_t wanted = static_cast<std::size_t>(std::atoi(opt["pairs"].c_str()));
  const double apart = std::atof(opt["apart_m"].c_str()), same = std::atof(opt["same_m"].c_str());
  std::mt19937 generator{static_cast<unsigned>(std::atoi(opt["seed"].c_str()))};
  std::uniform_int_distribution<std::size_t> pick{0U, scans.size() - 1U};
  std::vector<std::array<std::size_t, 3>> pairs;  // kind, A, B
  for (int kind = 0; kind < 2; ++kind) {
    std::size_t found = 0U;
    for (std::size_t tries = 0U; found < wanted && tries < 400U * wanted; ++tries) {
      const std::size_t a = pick(generator), b = pick(generator);
      if (scans[a].path < 5.0 || a == b) continue;
      const double distance = (scans[a].position - scans[b].position).norm();
      const bool far_in_time = std::llabs(scans[a].stamp - scans[b].stamp) >= 30'000'000'000LL;
      if (kind == 0 ? distance >= apart : (distance <= same && far_in_time)) { pairs.push_back({static_cast<std::size_t>(kind), a, b}); ++found; }
    }
  }
  const std::time_t started = std::time(nullptr);
  std::fprintf(stderr, "start %s  scans %zu path %.0f m pairs %zu threads %s\n", std::strtok(std::ctime(&started), "\n"), scans.size(), scans.back().path, pairs.size(), opt["threads"].c_str());
  std::vector<Result> results(pairs.size());
  std::atomic<std::size_t> next_pair{0U}, done{0U};
  std::mutex log_mutex;
  const auto work = [&] {
    for (std::size_t index = next_pair++; index < pairs.size(); index = next_pair++) {
      const Scan& a = scans[pairs[index][1]]; const Scan& b = scans[pairs[index][2]];
      // The submap around A: the scans before it a keyframe apart, the newest first, forty at most.
      PointPlaneMap3D map{PointPlaneMapConfig3D{.cell_m = config.scan_voxel_m, .maximum_points_per_cell = config.maximum_points_per_cell}};
      std::size_t keyframes = 0U; double last_path = a.path + config.keyframe_translation_m;
      for (std::size_t k = pairs[index][1] + 1U; k-- > 0U && keyframes < config.maximum_keyframes;) {
        if (last_path - scans[k].path < config.keyframe_translation_m) continue;
        std::vector<Eigen::Vector3d> world; world.reserve(scans[k].points.size());
        for (const Eigen::Vector3d& p : scans[k].points) world.push_back(scans[k].rotation * p + scans[k].position);
        map.insert(world, keyframes, scans[k].stamp);
        last_path = scans[k].path; ++keyframes;
      }
      PointPlaneRegistrationConfig3D registration_config;
      registration_config.maximum_correspondence_m = config.maximum_correspondence_m;
      registration_config.maximum_iterations = config.maximum_iterations;
      registration_config.convergence_translation_m = config.convergence_translation_m;
      registration_config.convergence_rotation_rad = config.convergence_rotation_rad;
      registration_config.robust_width_m = config.robust_width_m;
      const PointPlaneRegistration3D r = registerPointsToPlanes(map, b.points, a.position, b.rotation, registration_config);
      const double matched_points = r.matched_fraction * static_cast<double>(b.points.size());
      const bool accepted = r.converged && (r.matched_fraction >= config.minimum_matched_fraction || matched_points >= static_cast<double>(config.minimum_matched_points)) && r.residual_rms_m <= config.maximum_residual_rms_m;
      results[index] = {static_cast<int>(pairs[index][0]), a.stamp, b.stamp, (a.position - b.position).norm(), accepted, r.matched_fraction, r.residual_rms_m, r.information_per_point, (r.position - b.position).norm()};
      const std::size_t finished = ++done;
      if (finished % 100U == 0U) { const std::scoped_lock lock{log_mutex}; std::fprintf(stderr, "  done %zu of %zu elapsed %ld s\n", finished, pairs.size(), static_cast<long>(std::time(nullptr) - started)); }
    }
  };
  std::vector<std::thread> threads;
  for (int i = 0; i < std::atoi(opt["threads"].c_str()); ++i) threads.emplace_back(work);
  for (std::thread& thread : threads) thread.join();
  if (!opt["out"].empty()) {
    std::FILE* out = std::fopen(opt["out"].c_str(), "w");
    std::fprintf(out, "kind,stamp_a_ns,stamp_b_ns,apart_m,accepted,matched_fraction,residual_rms_m,information_per_point,pose_error_m\n");
    for (const Result& r : results) std::fprintf(out, "%s,%lld,%lld,%.2f,%d,%.3f,%.3f,%.4f,%.2f\n", r.kind == 0 ? "different" : "same", r.a, r.b, r.apart, r.accepted ? 1 : 0, r.matched, r.residual, r.information, r.error);
    std::fclose(out);
  }
  // What a true revisit looks like to the registration: the range the same-place pairs' accepted registrations
  // lie in (the matched fraction from its 5th percentile up, the residual up to its 95th), and the pairs of
  // different places that lie in it too.
  std::vector<double> true_matched, true_residual;
  for (const Result& r : results) if (r.kind == 1 && r.accepted && r.error <= 0.5) { true_matched.push_back(r.matched); true_residual.push_back(r.residual); }
  std::sort(true_matched.begin(), true_matched.end()); std::sort(true_residual.begin(), true_residual.end());
  const double matched_floor = true_matched.empty() ? 0.0 : true_matched[true_matched.size() * 5 / 100];
  const double residual_ceiling = true_residual.empty() ? 0.0 : true_residual[std::min(true_residual.size() - 1U, true_residual.size() * 95 / 100)];
  for (int kind = 0; kind < 2; ++kind) {
    std::size_t total = 0U, accepted = 0U, well_placed = 0U, observed = 0U, alike = 0U; std::vector<double> matched, residual, alike_apart;
    for (const Result& r : results) if (r.kind == kind) {
      ++total; matched.push_back(r.matched); residual.push_back(r.residual);
      if (r.accepted) { ++accepted; if (r.error <= 0.5) ++well_placed; if (r.information >= config.minimum_information_per_point) ++observed; }
      if (r.accepted && !true_matched.empty() && r.matched >= matched_floor && r.residual <= residual_ceiling) { ++alike; alike_apart.push_back(r.apart); }
    }
    std::sort(matched.begin(), matched.end()); std::sort(residual.begin(), residual.end()); std::sort(alike_apart.begin(), alike_apart.end());
    const auto at = [](const std::vector<double>& v, double share) { return v.empty() ? 0.0 : v[std::min(v.size() - 1U, static_cast<std::size_t>(share * static_cast<double>(v.size())))]; };
    const auto percent = [](std::size_t n, std::size_t of) { return of ? 100.0 * static_cast<double>(n) / static_cast<double>(of) : 0.0; };
    if (kind == 0) {
      std::printf("different places (at least %.0f m apart): %zu pairs; matched fraction p5 %.2f p50 %.2f p95 %.2f, residual p5 %.3f p50 %.3f p95 %.3f m\n", apart, total, at(matched, 0.05), at(matched, 0.5), at(matched, 0.95), at(residual, 0.05), at(residual, 0.5), at(residual, 0.95));
      std::printf("  accepted by the estimator's health rule: %zu (%.1f %%), %zu of them with every axis observed\n", accepted, percent(accepted, total), observed);
      if (true_matched.empty()) std::printf("  as good as a true revisit: not measured, the record has no revisit registered within 0.5 m of the truth\n");
      else std::printf("  as good as a true revisit (matched fraction %.2f or more, residual %.3f m or less): %zu (%.2f %%): false recognitions; the places %.0f to %.0f m apart\n", matched_floor, residual_ceiling, alike, percent(alike, total), at(alike_apart, 0.0), at(alike_apart, 1.0));
    } else {
      std::printf("the same place (within %.0f m, 30 s apart): %zu pairs; matched fraction p5 %.2f p50 %.2f p95 %.2f, residual p5 %.3f p50 %.3f p95 %.3f m\n", same, total, at(matched, 0.05), at(matched, 0.5), at(matched, 0.95), at(residual, 0.05), at(residual, 0.5), at(residual, 0.95));
      std::printf("  accepted by the estimator's health rule: %zu (%.1f %%), of them within 0.5 m of the truth %zu, with every axis observed %zu\n", accepted, percent(accepted, total), well_placed, observed);
    }
  }
  std::fprintf(stderr, "end elapsed %ld s\n", static_cast<long>(std::time(nullptr) - started));
  return 0;
}
