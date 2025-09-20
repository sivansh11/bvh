#ifndef BVH_BVH_HPP
#define BVH_BVH_HPP

#include <cstdint>
#include <vector>

#include "math/math.hpp"
#include "model/model.hpp"

namespace bvh {

struct node_t {
  math::vec3 min;
  uint32_t index; // 16 bytes
  math::vec3 max;
  uint32_t prim_count; // 16 bytes, total: 32 bytes
  bool is_leaf() const { return prim_count != 0; }
  math::aabb_t aabb() const { return math::aabb_t{min, max}; }
};
static_assert(sizeof(node_t) == 32, "sizeof(node_t) should be 32 bytes");

struct bvh_triangle_t {
  math::vec4 v0;
  math::vec4 v1;
  math::vec4 v2;
};

struct bvh_t {
  std::vector<node_t> nodes;
  std::vector<bvh_triangle_t> triangles;
  std::vector<uint32_t> prim_indices;
};

bvh_t build_bvh(const model::raw_mesh_t &mesh);

uint32_t depth_of_bvh(const bvh_t &bvh);
float cost_of_bvh(const bvh_t &bvh);

} // namespace bvh

#endif
