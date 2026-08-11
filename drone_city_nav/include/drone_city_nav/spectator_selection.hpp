#pragma once

#include <cstddef>
#include <optional>
#include <string_view>
#include <vector>

namespace drone_city_nav {

enum class SpectatorReselectionPolicy {
  kFirstLiving,
  kNextLiving,
};

[[nodiscard]] std::optional<SpectatorReselectionPolicy>
parseSpectatorReselectionPolicy(std::string_view value);

[[nodiscard]] std::string_view
spectatorReselectionPolicyName(SpectatorReselectionPolicy policy);

class SpectatorSelection {
public:
  SpectatorSelection(std::size_t vehicle_count, std::size_t initial_index,
                     SpectatorReselectionPolicy policy);

  [[nodiscard]] std::size_t currentIndex() const noexcept;
  [[nodiscard]] bool destroyed(std::size_t index) const;

  // Returns the replacement index only when the selected vehicle was destroyed.
  [[nodiscard]] std::optional<std::size_t> markDestroyed(std::size_t index);

private:
  [[nodiscard]] std::optional<std::size_t> findFirstLiving() const;
  [[nodiscard]] std::optional<std::size_t> findNextLiving() const;

  std::vector<bool> destroyed_;
  std::size_t current_index_{0U};
  SpectatorReselectionPolicy policy_{SpectatorReselectionPolicy::kFirstLiving};
};

} // namespace drone_city_nav
