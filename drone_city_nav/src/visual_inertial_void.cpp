#include "drone_city_nav/visual_inertial_void.hpp"

namespace drone_city_nav {

bool VisualInertialVoid::frame(const std::int64_t stamp_ns,
                               const std::int64_t previous_frame_stamp_ns,
                               const std::int64_t last_update_stamp_ns,
                               const bool few_features,
                               const bool past_unaided) noexcept {
  const bool void_frame =
      few_features && witness_ && stamp_ns - witness_stamp_ns_ <= kWitnessFreshNs;
  if (void_frame && past_unaided) {
    if (in_void_ && previous_frame_stamp_ns > last_update_stamp_ns) {
      seconds_ += static_cast<double>(stamp_ns - previous_frame_stamp_ns) * 1.0e-9;
    }
    in_void_ = true;
  } else if (few_features) {
    in_void_ = false;
  }
  return in_void_;
}

} // namespace drone_city_nav
