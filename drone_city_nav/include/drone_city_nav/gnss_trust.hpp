#pragma once

#include <cstdint>
#include <string_view>

namespace drone_city_nav {

// Whether the satellite signal is trusted, read from one measurement: its
// disagreement with the estimator over time (roadmap item 39, stages 1 and
// 2). A fix agrees when every reading of it stands inside its gate: the
// position and the velocity residuals against the estimator's prediction,
// each normalised by the two uncertainties together; the fix's agreement
// with itself (its velocity against the difference of its positions); the
// step of its position from the previous fix; the step of its clock. The
// signal is trusted once it has agreed for the window, and not for one fix
// less; a disagreeing fix ends the trust at once; the trust returns when the
// agreement has lasted the window again. No reading is remembered beyond
// the window: nothing says "it lied once" (the owner's rule, specification
// I3). A gate left at infinity never fails (the measuring flights of the
// law, before the gates had a source).
struct GnssTrustConfig {
  double window_s{5.0};
  double position_gate{1.0e300};     // normalised residual
  double velocity_gate{1.0e300};     // normalised residual
  double self_gate_mps{1.0e300};     // the fix's velocity against its own positions
  double jump_gate_m{1.0e300};       // the position's step between fixes
  double clock_step_gate_s{1.0e300}; // the fix's clock against the previous fix's
};

struct GnssFixReading {
  std::int64_t stamp_ns{0};
  bool valid{false}; // a 3D fix with finite numbers
  double position_residual_normalised{0.0};
  double velocity_residual_normalised{0.0};
  double self_disagreement_mps{0.0};
  double position_step_m{0.0};
  double clock_step_s{0.0};
};

struct GnssTrustVerdict {
  bool agreeing{false};                // this fix, inside every gate
  bool trusted{false};                 // the agreement has lasted the window
  double agreeing_s{0.0};              // how long the agreement has lasted
  std::string_view reason{"agreeing"}; // the first gate the fix failed, or "agreeing"
};

class GnssTrust {
public:
  explicit GnssTrust(const GnssTrustConfig& config) noexcept
      : config_{config} {
  }

  [[nodiscard]] GnssTrustVerdict read(const GnssFixReading& reading) noexcept;

  [[nodiscard]] bool trusted() const noexcept {
    return trusted_;
  }

private:
  GnssTrustConfig config_;
  bool agreeing_{false};
  bool trusted_{false};
  std::int64_t agreement_since_ns_{0};
};

} // namespace drone_city_nav
