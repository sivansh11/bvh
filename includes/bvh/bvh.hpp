#ifndef BVH_BVH_HPP
#define BVH_BVH_HPP

#include "math/aabb.hpp"

#include <limits>

namespace bvh {

#define FIRST_INDEX_BITS_SIZE 28
struct node_t {
  math::aabb_t aabb{};
  uint32_t is_leaf : 1;
  uint32_t primitive_count : 31;
  union as_t {
    struct leaf_t {
      uint32_t first_primitive_index : FIRST_INDEX_BITS_SIZE;
      uint32_t dummy : 32 - FIRST_INDEX_BITS_SIZE; // do not use
    } leaf;
    struct internal_t {
      uint32_t first_child_index : FIRST_INDEX_BITS_SIZE;
      uint32_t children_count : 32 - FIRST_INDEX_BITS_SIZE;
    } internal;
  } as;
  uint32_t parent_index;
  bool operator==(const node_t &other) const;
};
static_assert(sizeof(node_t), "sizeof(node_t) != 32");

enum class object_split_search_type_t : uint32_t {
  e_uniform_sah,
  e_binned_sah,
};

struct options_t {
  uint32_t o_min_primitives_per_node = 1;
  uint32_t o_max_primitives_per_node = std::numeric_limits<uint32_t>::max();
  object_split_search_type_t o_object_split_search_type =
      object_split_search_type_t::e_binned_sah;
  float o_primitive_intersection_cost = 1.1f;
  float o_node_intersection_cost = 1.f;
  uint32_t o_samples = 8;
};

struct bvh_t {
  std::vector<node_t> nodes;
  std::vector<uint32_t> primitive_indices;
};

bvh_t build_bvh2(math::aabb_t *aabbs, glm::vec3 *centers,
                 uint32_t primitive_count, const options_t &options);

float cost_of_bvh(const bvh_t &bvh, const options_t &options);
uint32_t depth_of_bvh(const bvh_t &bvh);

} // namespace bvh

#endif // !BVH_BVH_HPP
