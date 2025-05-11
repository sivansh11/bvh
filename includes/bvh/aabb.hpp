#ifndef BVH_AABB_HPP
#define BVH_AABB_HPP

#include <cassert>
#include <limits>
#include <ostream>

#include <glm/glm.hpp>
#define GLM_ENABLE_EXPERIMENTAL 1
#include <glm/gtx/string_cast.hpp>

namespace bvh {

static constexpr float infinity = std::numeric_limits<float>::max();

struct aabb_t {
  bool is_valid() const;
  glm::vec3 extent() const;
  float area() const;
  glm::vec3 center() const;
  aabb_t &grow(const glm::vec3 &point);
  aabb_t &grow(const aabb_t &aabb);

  glm::vec3 min{infinity};
  glm::vec3 max{-infinity};
};

} // namespace bvh

std::ostream &operator<<(std::ostream &o, const bvh::aabb_t &aabb);

#endif // !BVH_AABB_HPP
