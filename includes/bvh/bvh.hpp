#ifndef BVH_BVH_HPP
#define BVH_BVH_HPP

#include <cstdint>
#include <vector>

#include "math/aabb.hpp"
#include "math/math.hpp"
#include "model/model.hpp"

namespace bvh {

struct node_t {
  math::vec3   min;
  uint32_t     index;  // left first or index of primitive
  math::vec3   max;
  uint32_t     prim_count;
  bool         is_leaf() const { return prim_count != 0; }
  math::aabb_t aabb() const { return math::aabb_t{min, max}; }
};
static_assert(sizeof(node_t) == 32, "sizeof(node_t) should be 32 bytes");

struct bvh_triangle_t {
  math::vec4   v0;
  math::vec4   v1;
  math::vec4   v2;
  math::aabb_t aabb() const {
    return math::aabb_t{}.grow(v0).grow(v1).grow(v2);
  }
  math::vec3 center() const {
    math::vec3 _v0 = v0;
    math::vec3 _v1 = v1;
    math::vec3 _v2 = v2;
    return (_v0 + _v1 + _v2) / 3.f;
  }
  float area() const {
    math::vec3 e1 = v1 - v0;
    math::vec3 e2 = v2 - v0;
    return math::cross(e1, e2).length() * 0.5f;
  }
  std::pair<math::aabb_t, math::aabb_t> split(uint32_t axis, float position) const;
};

struct bvh_t {
  std::vector<node_t>         nodes;
  std::vector<uint32_t>       prim_indices;
  std::vector<bvh_triangle_t> triangles;
};

bvh_t build_bvh(const model::raw_mesh_t &mesh);

uint32_t depth_of_bvh(const bvh_t &bvh);
float    cost_of_bvh(const bvh_t &bvh);

}  // namespace bvh

#endif
