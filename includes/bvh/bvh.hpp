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
  math::vec4                            v0;
  math::vec4                            v1;
  math::vec4                            v2;
  math::aabb_t                          aabb() const;
  math::vec3                            center() const;
  float                                 area() const;
  std::pair<math::aabb_t, math::aabb_t> split(uint32_t axis,
                                              float    position) const;
};

struct bvh_t {
  std::vector<node_t>         nodes;
  std::vector<uint32_t>       prim_indices;
  std::vector<bvh_triangle_t> triangles;
};

std::vector<bvh_triangle_t> triangles_from_mesh(const model::raw_mesh_t &mesh);
void                        presplit_remove_indirection(bvh_t                       &bvh,
                                                        const std::vector<uint32_t> &tri_indices);
void                        presplit_remove_duplicates(bvh_t &bvh);
std::pair<std::vector<math::aabb_t>, std::vector<uint32_t>> presplit(
    const std::vector<bvh_triangle_t> &triangles, float split_factor = 0.3f);
bvh_t build_bvh_binned_sah(const std::vector<math::aabb_t> &aabbs,
                           uint32_t                         num_samples    = 8,
                           uint32_t                         min_primitives = 1,
                           uint32_t                         max_primitives = 1);
bvh_t build_bvh_ploc(const std::vector<math::aabb_t> &aabbs,
                     uint32_t grid_dim = 1024, uint32_t log_bits = 10,
                     uint32_t search_radius = 15);
// create bvh with default parameters
bvh_t build_bvh(const model::raw_mesh_t &mesh);

uint32_t depth_of_bvh(const bvh_t &bvh);
float    cost_of_bvh(const bvh_t &bvh);

}  // namespace bvh

#endif
