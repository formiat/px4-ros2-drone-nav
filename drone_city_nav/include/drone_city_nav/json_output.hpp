#pragma once

#include <cmath>
#include <ostream>
#include <sstream>
#include <type_traits>
#include <utility>

namespace drone_city_nav {

// JSON does not permit NaN or infinities. This stream preserves the familiar
// ostream interface while mapping every non-finite floating-point value to
// JSON null at the serialization boundary.
class JsonOutputStream final {
public:
  JsonOutputStream& operator<<(std::ostream& (*manipulator)(std::ostream&)) {
    stream_ << manipulator;
    return *this;
  }

  JsonOutputStream& operator<<(std::ios_base& (*manipulator)(std::ios_base&)) {
    stream_ << manipulator;
    return *this;
  }

  template<typename Value>
    requires std::is_floating_point_v<std::remove_cvref_t<Value>>
  JsonOutputStream& operator<<(Value value) {
    if (std::isfinite(value)) {
      stream_ << value;
    } else {
      stream_ << "null";
    }
    return *this;
  }

  template<typename Value>
    requires(!std::is_floating_point_v<std::remove_cvref_t<Value>>)
  JsonOutputStream& operator<<(Value&& value) {
    stream_ << std::forward<Value>(value);
    return *this;
  }

  [[nodiscard]] std::string str() const {
    return stream_.str();
  }

private:
  std::ostringstream stream_;
};

} // namespace drone_city_nav
