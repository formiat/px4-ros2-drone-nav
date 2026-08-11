#include "drone_city_nav/spectator_selection.hpp"

#include <stdexcept>

namespace drone_city_nav {

std::optional<SpectatorReselectionPolicy>
parseSpectatorReselectionPolicy(const std::string_view value) {
  if (value == "first_living") {
    return SpectatorReselectionPolicy::kFirstLiving;
  }
  if (value == "next_living") {
    return SpectatorReselectionPolicy::kNextLiving;
  }
  return std::nullopt;
}

std::string_view
spectatorReselectionPolicyName(const SpectatorReselectionPolicy policy) {
  switch (policy) {
    case SpectatorReselectionPolicy::kFirstLiving:
      return "first_living";
    case SpectatorReselectionPolicy::kNextLiving:
      return "next_living";
  }
  throw std::logic_error{"unknown spectator reselection policy"};
}

SpectatorSelection::SpectatorSelection(const std::size_t vehicle_count,
                                       const std::size_t initial_index,
                                       const SpectatorReselectionPolicy policy)
    : destroyed_(vehicle_count, false),
      current_index_{initial_index},
      policy_{policy} {
  if (vehicle_count == 0U) {
    throw std::invalid_argument{"spectator vehicle list must not be empty"};
  }
  if (initial_index >= vehicle_count) {
    throw std::out_of_range{"initial spectator index is out of range"};
  }
}

std::size_t SpectatorSelection::currentIndex() const noexcept {
  return current_index_;
}

bool SpectatorSelection::destroyed(const std::size_t index) const {
  return destroyed_.at(index);
}

std::optional<std::size_t> SpectatorSelection::markDestroyed(const std::size_t index) {
  destroyed_.at(index) = true;
  if (index != current_index_) {
    return std::nullopt;
  }

  const std::optional<std::size_t> replacement =
      policy_ == SpectatorReselectionPolicy::kFirstLiving ? findFirstLiving()
                                                          : findNextLiving();
  if (replacement.has_value()) {
    current_index_ = replacement.value();
  }
  return replacement;
}

std::optional<std::size_t> SpectatorSelection::findFirstLiving() const {
  for (std::size_t index = 0U; index < destroyed_.size(); ++index) {
    if (!destroyed_[index]) {
      return index;
    }
  }
  return std::nullopt;
}

std::optional<std::size_t> SpectatorSelection::findNextLiving() const {
  for (std::size_t offset = 1U; offset <= destroyed_.size(); ++offset) {
    const std::size_t index = (current_index_ + offset) % destroyed_.size();
    if (!destroyed_[index]) {
      return index;
    }
  }
  return std::nullopt;
}

} // namespace drone_city_nav
