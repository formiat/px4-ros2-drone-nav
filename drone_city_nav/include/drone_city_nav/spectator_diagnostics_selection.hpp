#pragma once

#include <string>
#include <string_view>
#include <utility>

namespace drone_city_nav {

class SpectatorDiagnosticsSelection final {
public:
  SpectatorDiagnosticsSelection() = default;

  explicit SpectatorDiagnosticsSelection(std::string vehicle_id)
      : vehicle_id_{std::move(vehicle_id)},
        selected_{vehicle_id_.empty()} {
  }

  [[nodiscard]] bool gated() const noexcept {
    return !vehicle_id_.empty();
  }

  [[nodiscard]] bool selected() const noexcept {
    return selected_;
  }

  bool select(std::string_view vehicle_id) noexcept {
    const bool next_selected = vehicle_id_.empty() || vehicle_id == vehicle_id_;
    const bool changed = next_selected != selected_;
    selected_ = next_selected;
    return changed;
  }

private:
  std::string vehicle_id_;
  bool selected_{true};
};

} // namespace drone_city_nav
