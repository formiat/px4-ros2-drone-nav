#pragma once

#include "drone_city_nav/portal_graph.hpp"
#include "drone_city_nav/types.hpp"

#include <compare>
#include <cstddef>
#include <map>
#include <span>
#include <vector>

namespace drone_city_nav::detail {

struct OrientedPassageCandidate {
  std::size_t passage_index{0U};
  bool reversed{false};

  [[nodiscard]] bool
  operator==(const OrientedPassageCandidate&) const noexcept = default;
};

class PassageEntrySpatialIndex {
public:
  PassageEntrySpatialIndex(std::span<const PassageTraversalEdge> passages,
                           double bucket_size_m);

  [[nodiscard]] std::vector<OrientedPassageCandidate> near(const Point3& point) const;

private:
  struct Bucket {
    int x{0};
    int y{0};
    int z{0};

    [[nodiscard]] auto operator<=>(const Bucket&) const noexcept = default;
  };

  [[nodiscard]] Bucket bucketFor(const Point3& point) const noexcept;

  double bucket_size_m_{1.0};
  std::map<Bucket, std::vector<OrientedPassageCandidate>> entries_;
};

} // namespace drone_city_nav::detail
