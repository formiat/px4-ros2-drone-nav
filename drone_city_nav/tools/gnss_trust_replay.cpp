// Roadmap item 39: the trust law (gnss_trust.hpp) replayed over the readings a
// flight recorded (the GNSS_TRUST lines of ros_drone_nav.log, one a second)
// under the gates given, with a synthetic disagreement injected for a stretch:
// prints how many recorded readings fail each gate, when the trust is lost after
// the fault begins and when it returns after the fault ends (the reaction time
// T, at the lines' one-second resolution). The law is the production one, not a
// copy of it.
//
//   gnss_trust_replay LOG --position-gate 0.70 --velocity-gate 1.26 --self-gate 2.0
//       --jump-gate 0.16 --clock-gate 0.072 [--fault-at 300 --fault-s 60
//       --fault-normalised 5]

#include "drone_city_nav/gnss_trust.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

double field(const std::string& line, std::string_view key, const double fallback) {
  const auto at = line.find(key);
  if (at == std::string::npos) {
    return fallback;
  }
  return std::strtod(line.c_str() + at + key.size(), nullptr);
}

struct Recorded {
  double stamp_s;
  drone_city_nav::GnssFixReading reading;
};

} // namespace

int main(const int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "usage: gnss_trust_replay LOG [--position-gate G] [--velocity-gate G] "
                 "[--self-gate MPS] [--jump-gate M] [--clock-gate S] [--window S] "
                 "[--fault-at S] [--fault-s S] [--fault-normalised N]\n");
    return 2;
  }
  drone_city_nav::GnssTrustConfig config;
  double fault_at_s = -1.0;
  double fault_s = 60.0;
  double fault_normalised = 5.0;
  for (int index = 2; index + 1 < argc; index += 2) {
    const std::string_view name{argv[index]};
    const double value = std::strtod(argv[index + 1], nullptr);
    if (name == "--position-gate") {
      config.position_gate = value;
    } else if (name == "--velocity-gate") {
      config.velocity_gate = value;
    } else if (name == "--self-gate") {
      config.self_gate_mps = value;
    } else if (name == "--jump-gate") {
      config.jump_gate_m = value;
    } else if (name == "--clock-gate") {
      config.clock_step_gate_s = value;
    } else if (name == "--window") {
      config.window_s = value;
    } else if (name == "--fault-at") {
      fault_at_s = value;
    } else if (name == "--fault-s") {
      fault_s = value;
    } else if (name == "--fault-normalised") {
      fault_normalised = value;
    } else {
      std::fprintf(stderr, "unknown option %s\n", argv[index]);
      return 2;
    }
  }

  std::ifstream log{argv[1]};
  if (!log) {
    std::fprintf(stderr, "cannot read %s\n", argv[1]);
    return 1;
  }
  std::vector<Recorded> recorded;
  for (std::string line; std::getline(log, line);) {
    if (line.find("GNSS_TRUST trusted=") == std::string::npos) {
      continue;
    }
    const auto open = line.find('[', line.find("[INFO]") + 6U);
    if (open == std::string::npos) {
      continue;
    }
    Recorded sample{};
    sample.stamp_s = std::strtod(line.c_str() + open + 1U, nullptr);
    sample.reading.stamp_ns =
        static_cast<std::int64_t>(std::llround(sample.stamp_s * 1.0e9));
    sample.reading.valid = field(line, "fix_type=", 0.0) >= 3.0;
    sample.reading.position_residual_normalised = field(line, " normalised=", 0.0);
    sample.reading.velocity_residual_normalised =
        field(line, "velocity_normalised=", 0.0);
    sample.reading.self_disagreement_mps = field(line, "self_mps=", 0.0);
    sample.reading.position_step_m = field(line, "step_m=", 0.0);
    sample.reading.clock_step_s = field(line, "clock_s=", 0.0);
    recorded.push_back(sample);
  }
  if (recorded.empty()) {
    std::fprintf(stderr, "no GNSS_TRUST lines in %s\n", argv[1]);
    return 1;
  }
  const double start_s = recorded.front().stamp_s;

  // The recorded readings alone: how many fail each gate under the law.
  {
    drone_city_nav::GnssTrust trust{config};
    std::size_t disagreeing = 0;
    std::size_t trusted = 0;
    for (const Recorded& sample : recorded) {
      const auto verdict = trust.read(sample.reading);
      disagreeing += verdict.agreeing ? 0U : 1U;
      trusted += verdict.trusted ? 1U : 0U;
    }
    std::printf("recorded: %zu readings over %.0f s, %zu disagreeing, trusted %zu\n",
                recorded.size(), recorded.back().stamp_s - start_s, disagreeing,
                trusted);
  }
  if (fault_at_s < 0.0) {
    return 0;
  }

  // The same readings with a synthetic disagreement from fault_at for fault_s.
  drone_city_nav::GnssTrust trust{config};
  const double fault_end_s = fault_at_s + fault_s;
  double lost_at_s = -1.0;
  double returned_at_s = -1.0;
  bool trusted_before_fault = false;
  for (const Recorded& sample : recorded) {
    const double t = sample.stamp_s - start_s;
    drone_city_nav::GnssFixReading reading = sample.reading;
    const bool faulted = t >= fault_at_s && t < fault_end_s;
    if (faulted) {
      reading.position_residual_normalised = fault_normalised;
    }
    const auto verdict = trust.read(reading);
    if (t < fault_at_s) {
      trusted_before_fault = verdict.trusted;
    } else if (lost_at_s < 0.0 && !verdict.trusted) {
      lost_at_s = t;
    } else if (lost_at_s >= 0.0 && t >= fault_end_s && returned_at_s < 0.0 &&
               verdict.trusted) {
      returned_at_s = t;
    }
  }
  std::printf(
      "fault at %.0f s for %.0f s (normalised %.1f): trusted before %s, lost %s, "
      "returned %s\n",
      fault_at_s, fault_s, fault_normalised, trusted_before_fault ? "yes" : "no",
      lost_at_s < 0.0 ? "never"
                      : (std::to_string(lost_at_s - fault_at_s) + " s after").c_str(),
      returned_at_s < 0.0
          ? "never"
          : (std::to_string(returned_at_s - fault_end_s) + " s after the end").c_str());
  return 0;
}
