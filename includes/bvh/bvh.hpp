#ifndef CORE_BVH_HPP
#define CORE_BVH_HPP

#include "math/aabb.hpp"

#include <limits>

#include <cstdint>

namespace core {

namespace bvh {

static constexpr uint32_t invalid_index = std::numeric_limits<uint32_t>::max();

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

  bool operator==(const node_t &other) const {
    return std::memcmp(this, &other, sizeof(node_t));
  }
};

static_assert(sizeof(node_t) == 32, "size of node exceeds 32");

struct bvh_t {
  std::vector<node_t> nodes;
  std::vector<uint32_t> primitive_indices;
};

enum class object_split_search_type_t {
  e_longest_axis_division,
  e_full_sweep_sah,
  e_uniform_sah,
  e_binned_sah,
};

struct options_t {
  uint32_t o_min_primitive_count = 1;
  uint32_t o_max_primitive_count = std::numeric_limits<uint32_t>::max();
  object_split_search_type_t o_object_split_search_type =
      object_split_search_type_t::e_binned_sah;
  float o_primitive_intersection_cost = 1.1f;
  float o_node_intersection_cost = 1.f;
  uint32_t o_samples = 8;
};

float cost_of_bvh(const bvh_t &bvh, const options_t &options);
uint32_t depth_of_bvh(const bvh_t &bvh);

bvh_t build_bvh2(math::aabb_t *aabbs, math::vec3 *centers,
                 uint32_t primitive_count, const options_t &options);
void collapse_nodes(bvh_t &bvh, const options_t &options);

} // namespace bvh

} // namespace core

#endif
