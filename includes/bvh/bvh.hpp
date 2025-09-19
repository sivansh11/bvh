#ifndef BVH_BVH_HPP
#define BVH_BVH_HPP

#include <cstdint>
#include <vector>

#include "math/aabb.hpp"
#include "math/math.hpp"
#include "model/model.hpp"

namespace bvh {

struct node_t {
  math::vec3 min;
  uint32_t   index;  // left first or index of child
  math::vec3 max;
  uint32_t   prim_count;

  math::aabb_t aabb() const { return math::aabb_t{min, max}; }
};
static_assert(sizeof(node_t) == 32, "sizeof(node_t) should be 32 bytes");

struct bvh_triangle_t {
  math::vec4 v0;
  math::vec4 v1;
  math::vec4 v2;
};

struct bvh_t {
  std::vector<node_t>         nodes;
  std::vector<uint32_t>       indices;
  std::vector<bvh_triangle_t> triangles;
};

bvh_t build_bvh(const model::raw_mesh_t &mesh);

}  // namespace bvh

#endif
