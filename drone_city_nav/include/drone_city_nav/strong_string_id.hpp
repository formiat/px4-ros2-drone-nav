#pragma once

#include <compare>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>

namespace drone_city_nav {

template<typename Tag> class StrongStringId final {
public:
  StrongStringId() = default;

  StrongStringId(const char* value)
      : value_{value} {
  }

  StrongStringId(std::string value)
      : value_{std::move(value)} {
  }

  StrongStringId(const std::string_view value)
      : value_{value} {
  }

  [[nodiscard]] bool empty() const noexcept {
    return value_.empty();
  }

  [[nodiscard]] const std::string& value() const noexcept {
    return value_;
  }

  [[nodiscard]] const char* c_str() const noexcept {
    return value_.c_str();
  }

  [[nodiscard]] auto operator<=>(const StrongStringId&) const noexcept = default;

  friend std::ostream& operator<<(std::ostream& stream, const StrongStringId& id) {
    return stream << id.value_;
  }

private:
  std::string value_;
};

} // namespace drone_city_nav
