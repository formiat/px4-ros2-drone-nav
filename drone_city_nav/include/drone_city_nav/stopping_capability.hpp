#pragma once

namespace drone_city_nav {

struct StoppingCapability {
  double maximum_commanded_horizontal_deceleration_mps2{4.0};
  double guaranteed_horizontal_deceleration_mps2{4.0};
  double guaranteed_vertical_deceleration_mps2{2.0};
  double reaction_latency_s{0.1};
};

[[nodiscard]] constexpr bool
stoppingCapabilityIsValid(const StoppingCapability& capability) noexcept {
  return capability.maximum_commanded_horizontal_deceleration_mps2 > 0.0 &&
         capability.guaranteed_horizontal_deceleration_mps2 > 0.0 &&
         capability.guaranteed_horizontal_deceleration_mps2 <=
             capability.maximum_commanded_horizontal_deceleration_mps2 &&
         capability.guaranteed_vertical_deceleration_mps2 > 0.0 &&
         capability.reaction_latency_s >= 0.0;
}

} // namespace drone_city_nav
