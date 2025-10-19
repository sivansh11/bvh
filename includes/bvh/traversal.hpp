#ifndef BVH_TRAVERSAL_HPP
#define BVH_TRAVERSAL_HPP

#include "bvh.hpp"
#include "math/math.hpp"

namespace bvh {

inline float safe_inverse(float x) {
  static constexpr float epsilon = std::numeric_limits<float>::epsilon();
  if (std::abs(x) <= epsilon) {
    return x >= 0 ? 1.f / epsilon : -1.f / epsilon;
  }
  return 1.f / x;
}

static const uint32_t null_index = std::numeric_limits<uint32_t>::max();

struct hit_t {
  bool     did_intersect() { return prim_index != null_index; }
  uint32_t prim_index = null_index;
  float    t          = 1e30;
  float    u, v;
  uint32_t node_intersections     = 0;
  uint32_t triangle_intersections = 0;
};

struct triangle_hit_t {
  bool  did_intersect() { return _did_intersect; }
  float t, u, v;
  bool  _did_intersect;
};

struct aabb_hit_t {
  bool  did_intersect() { return tmin <= tmax; }
  float tmin, tmax;
};

struct ray_t {
  static ray_t create(math::vec3 origin, math::vec3 direction) {
    ray_t ray;
    ray.origin    = origin;
    ray.direction = direction;
    ray.inverse_direction =
        math::vec3(safe_inverse(direction.x), safe_inverse(direction.y),
                   safe_inverse(direction.z));
    ray.tmax = 1e30;
    ray.tmin = 0.0001;
    return ray;
  }
  math::vec3 origin, direction, inverse_direction;
  float      tmax, tmin;
};

triangle_hit_t intersect_triangle(const math::triangle_t triangle,
                                  const ray_t            ray);
aabb_hit_t     intersect_aabb(const math::vec3 _min, const math::vec3 _max,
                              const ray_t ray);
hit_t          intersect_bvh(bvh::node_t *nodes, uint32_t *indices,
                             math::triangle_t *triangles, ray_t ray);

}  // namespace bvh

#endif
