#ifndef BVH_TRAVERSAL_HPP
#define BVH_TRAVERSAL_HPP

#include "bvh.hpp"
#include "bvh/ray.hpp"

#include "math/triangle.hpp"

namespace bvh {

struct ray_data_t {
  static ray_data_t create(const bvh::ray_t &ray);
  math::vec3 origin, direction;
  math::vec3 inverse_direction;
  float tmin, tmax;
};

struct aabb_intersection_t {
  bool did_intersect() { return tmin <= tmax; }
  float tmin, tmax;
};

struct triangle_intersection_t {
  bool did_intersect() { return is_intersect; }
  bool is_intersect;
  float t, u, v, w;
};

struct hit_t {
  bool did_intersect_blas() { return primitive_index != null_index; }
  bool did_intersect() {
    return primitive_index != null_index && instance_id != null_index;
  }
  uint32_t instance_id = null_index;
  uint32_t primitive_index = null_index;
  float t = math::infinity; // ray hit t
  float u, v, w;
  // math::mat4 inverse_transform;
  uint32_t node_intersection_count = 0;
  uint32_t primitive_intersection_count = 0;
};

static constexpr hit_t null_hit{.primitive_index = null_index};

triangle_intersection_t triangle_intersect(const math::triangle_t &triangle,
                                           const ray_data_t &ray_data);

aabb_intersection_t aabb_intersect(const math::aabb_t &aabb,
                                   const ray_data_t &ray_data);

hit_t traverse(uint32_t *primitive_indices, node_t *nodes,
               math::triangle_t *triangles, ray_data_t &ray_data);

} // namespace bvh

#endif // !BVH_TRAVERSAL_HPP
