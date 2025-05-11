#include "bvh/aabb.hpp"

namespace bvh {

bool aabb_t::is_valid() const {
  return min != glm::vec3{infinity} && max != glm::vec3{-infinity};
}
glm::vec3 aabb_t::extent() const {
  assert(is_valid());
  return max - min;
}
float aabb_t::area() const {
  assert(is_valid());
  const glm::vec3 e = extent();
  return (e.x * e.y + e.y * e.z + e.z * e.x);
}
glm::vec3 aabb_t::center() const {
  assert(is_valid());
  return (max + min) / 2.f;
}
aabb_t &aabb_t::grow(const glm::vec3 &point) {
  min = glm::min(min, point);
  max = glm::max(max, point);
  return *this;
}
aabb_t &aabb_t::grow(const aabb_t &aabb) {
  min = glm::min(min, aabb.min);
  max = glm::max(max, aabb.max);
  return *this;
}

} // namespace bvh

std::ostream &operator<<(std::ostream &o, const bvh::aabb_t &aabb) {
  o << "aabb:\n\tmin: " << glm::to_string(aabb.min)
    << "\n\tmax: " << glm::to_string(aabb.max);
  return o;
}
