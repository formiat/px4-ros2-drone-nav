#pragma once

namespace drone_city_nav {

class MappingLifecycle {
public:
  explicit MappingLifecycle(double activation_altitude_m);

  void updateArmed(bool armed) noexcept;
  [[nodiscard]] bool updateAltitude(double altitude_m, bool valid) noexcept;
  [[nodiscard]] bool active() const noexcept;

private:
  double activation_altitude_m_;
  bool armed_{false};
  bool armed_seen_{false};
  bool active_{false};
};

} // namespace drone_city_nav
