#include "bvh/bvh.hpp"
#include "geometry/aabb.hpp"
#include <algorithm>
#include <cassert>
#include <cstring>
#include <iterator>
#include <limits>
#include <span>
#include <stdexcept>

namespace bvh {

struct split_t {
  uint32_t axis = std::numeric_limits<uint32_t>::max(); // undefined
  float position = geometry::infinity;
  float cost = geometry::infinity;
};

struct bin_t {
  geometry::aabb_t aabb{};
  uint32_t primitive_count = 0;
};

uint32_t number_of_bits_required_for_number(uint32_t n) {
  uint32_t r = 1;
  if (n >> 16) {
    r += 16;
    n >>= 16;
  }
  if (n >> 8) {
    r += 8;
    n >>= 8;
  }
  if (n >> 4) {
    r += 4;
    n >>= 4;
  }
  if (n >> 2) {
    r += 2;
    n >>= 2;
  }
  if (n - 1)
    ++r;
  return r;
}

float cost_of_node(const bvh_t &bvh, uint32_t node_index,
                   const options_t &options) {
  const node_t &node = bvh.nodes[node_index];
  if (node.is_leaf)
    return options.o_primitive_intersection_cost * node.primitive_count;

  float cost = 0;
  for (uint32_t i = 0; i < node.as.internal.children_count; i++) {
    const node_t &child = bvh.nodes[node.as.internal.first_child_index + i];
    cost += child.aabb.area() * cost_of_node(bvh, node_index, options);
  }
  return options.o_node_intersection_cost + (cost / node.aabb.area());
}

float greedy_cost_of_node(uint32_t left_count, uint32_t right_count,
                          float left_aabb_area, float right_aabb_area,
                          float parent_aabb_area, const options_t &options) {
  return options.o_node_intersection_cost +
         ((left_aabb_area * options.o_primitive_intersection_cost * left_count +
           right_aabb_area * options.o_primitive_intersection_cost *
               right_count) /
          parent_aabb_area);
}

uint32_t depth_of_node(const bvh_t &bvh, uint32_t node_id) {
  const node_t &node = bvh.nodes[node_id];
  if (node.is_leaf) {
    return 1;
  } else {
    uint32_t children_max_depth = 0;
    for (uint32_t i = 0; i < node.as.internal.children_count; i++) {
      children_max_depth =
          std::max(children_max_depth,
                   depth_of_node(bvh, node.as.internal.first_child_index + i));
    }
    return children_max_depth + 1;
  }
  throw std::runtime_error("shouldnt reach here");
}

bool node_t::operator==(const node_t &other) const {
  return std::memcmp(this, &other, sizeof(node_t));
}

node_t new_node(uint32_t first_primitive_index, uint32_t primitive_count) {
  node_t node{};
  assert(!node.aabb.is_valid()); // node's aabb should not be valid initially
  node.is_leaf = true;
  node.as.leaf.first_primitive_index = first_primitive_index;
  node.primitive_count = primitive_count;
  return node;
}

void update_node_bounds(bvh_t &bvh, uint32_t node_index,
                        geometry::aabb_t *aabbs) {
  node_t &node = bvh.nodes[node_index];
  if (node.is_leaf) {
    node.aabb = {};
    for (uint32_t i = 0; i < node.primitive_count; i++) {
      node.aabb.grow(
          aabbs[bvh.primitive_indices[node.as.leaf.first_primitive_index + i]]);
    }
  } else {
    node.aabb = {};
    for (auto &child :
         std::span(bvh.nodes.begin() + node.as.internal.first_child_index,
                   node.as.internal.children_count)) {
      node.aabb.grow(child.aabb);
    }
  }
}

void try_split_node(bvh_t &bvh, uint32_t node_index, geometry::aabb_t *aabbs,
                    glm::vec3 *centers, const options_t &options);

uint32_t partition_node_indices(bvh_t &bvh, uint32_t node_index,
                                const split_t &split, glm::vec3 *centers) {
  node_t &node = bvh.nodes[node_index];
  auto middle = std::partition(
      bvh.primitive_indices.begin() + node.as.leaf.first_primitive_index,
      bvh.primitive_indices.begin() + node.as.leaf.first_primitive_index +
          node.primitive_count,
      [&](uint32_t index) {
        return centers[index][split.axis] < split.position;
      });
  return std::distance(bvh.primitive_indices.begin(), middle);
}

bool split_node(bvh_t &bvh, uint32_t node_index, const split_t &split,
                geometry::aabb_t *aabbs, glm::vec3 *centers,
                const options_t &options) {
  if (split.axis == std::numeric_limits<uint32_t>::max())
    return false;

  node_t &node = bvh.nodes[node_index];

  uint32_t right_first_primitive_index =
      partition_node_indices(bvh, node_index, split, centers);

  if (node.as.leaf.first_primitive_index == right_first_primitive_index |
      node.as.leaf.first_primitive_index + node.primitive_count ==
          right_first_primitive_index)
    return false;

  node_t left = new_node(node.as.leaf.first_primitive_index,
                         right_first_primitive_index -
                             node.as.leaf.first_primitive_index);
  node_t right =
      new_node(right_first_primitive_index, node.as.leaf.first_primitive_index +
                                                node.primitive_count -
                                                right_first_primitive_index);

  node.is_leaf = false;

  if (number_of_bits_required_for_number(bvh.nodes.size()) >
      FIRST_INDEX_BITS_SIZE)
    throw std::runtime_error(
        "number doesnt fit in FIRST_INDEX_BITS_SIZE, node index overflow");

  node.as.internal.first_child_index = bvh.nodes.size();
  node.as.internal.first_child_index = 2;

  uint32_t left_node_index = bvh.nodes.size();
  bvh.nodes.push_back(left);

  uint32_t right_node_index = bvh.nodes.size();
  bvh.nodes.push_back(right);

  update_node_bounds(bvh, left_node_index, aabbs);
  update_node_bounds(bvh, right_node_index, aabbs);

  try_split_node(bvh, left_node_index, aabbs, centers, options);
  try_split_node(bvh, right_node_index, aabbs, centers, options);

  return true;
}

split_t find_best_object_split(const bvh_t &bvh, uint32_t node_index,
                               geometry::aabb_t *aabbs, glm::vec3 *centers,
                               const options_t &options) {
  const node_t &node = bvh.nodes[node_index];
  split_t best_split{};

  switch (options.o_object_split_search_type) {
  case object_split_search_type_t::e_binned_sah: {
    geometry::aabb_t split_bounds{};
    for (uint32_t i = 0; i < node.primitive_count; i++)
      split_bounds.grow(
          centers[bvh.primitive_indices[node.as.leaf.first_primitive_index +
                                        i]]);
    for (uint32_t axis = 0; axis < 3; axis++) {
      if (split_bounds.max[axis] == split_bounds.min[axis])
        continue;

      bin_t *p_bins =
          reinterpret_cast<bin_t *>(alloca(sizeof(bin_t) * options.o_samples));
      std::memset(p_bins, 0, sizeof(bin_t) * options.o_samples);
      std::span<bin_t> bins{p_bins, options.o_samples};

      float scale = static_cast<float>(options.o_samples) /
                    (split_bounds.max[axis] - split_bounds.min[axis]);

      for (uint32_t i = 0; i < node.primitive_count; i++) {
        uint32_t bin_id = std::min(
            options.o_samples - 1,
            static_cast<uint32_t>(
                centers[bvh.primitive_indices
                            [node.as.leaf.first_primitive_index + i]][axis] -
                split_bounds.min[axis] * scale));
        bins[bin_id].primitive_count++;
        bins[bin_id].aabb.grow(
            aabbs[bvh.primitive_indices[node.as.leaf.first_primitive_index +
                                        i]]);
      }

      float *left_area = reinterpret_cast<float *>(
          alloca(sizeof(float) * options.o_samples - 1));
      float *right_area = reinterpret_cast<float *>(
          alloca(sizeof(float) * options.o_samples - 1));
      uint32_t *left_count = reinterpret_cast<uint32_t *>(
          alloca(sizeof(uint32_t) * options.o_samples - 1));
      uint32_t *right_count = reinterpret_cast<uint32_t *>(
          alloca(sizeof(uint32_t) * options.o_samples - 1));
      geometry::aabb_t left_aabb{};
      geometry::aabb_t right_aabb{};
      uint32_t left_sum = 0;
      uint32_t right_sum = 0;
      for (uint32_t i = 0; i < options.o_samples - 1; i++) {
        left_sum += bins[i].primitive_count;
        left_count[i] = left_sum;
        left_aabb.grow(bins[i].aabb);
        left_area[i] = left_aabb.area();

        right_sum += bins[options.o_samples - i - 1].primitive_count;
        right_count[options.o_samples - i - 2] = right_sum;
        right_aabb.grow(bins[options.o_samples - i - 1].aabb);
        right_area[options.o_samples - i - 2] = right_aabb.area();
      }
      scale = (split_bounds.max[axis] - split_bounds.min[axis]) /
              static_cast<float>(options.o_samples);
      for (uint32_t i = 0; i < options.o_samples - 1; i++) {
        float cost =
            greedy_cost_of_node(left_count[i], right_count[i], left_area[i],
                                right_area[i], node.aabb.area(), options);
        if (cost < best_split.cost) {
          best_split.cost = cost;
          best_split.axis = axis;
          best_split.position = split_bounds.min[axis] + scale * (i + 1);
        }
      }
    }
  } break;
  case object_split_search_type_t::e_uniform_sah:
  default:
    throw std::runtime_error("unknown split type");
  }

  return best_split;
}

void try_split_node(bvh_t &bvh, uint32_t node_index, geometry::aabb_t *aabbs,
                    glm::vec3 *centers, const options_t &options) {
  node_t &node = bvh.nodes[node_index];
  if (node.primitive_count <= options.o_min_primitives_per_node)
    return;
  const split_t split =
      find_best_object_split(bvh, node_index, aabbs, centers, options);

  if (node.primitive_count > options.o_max_primitives_per_node) {
    bool did_split =
        split_node(bvh, node_index, split, aabbs, centers, options);
    // TODO: handle fallback split (maybe largest axis ?)
  } else {
    float no_split_cost = cost_of_node(bvh, node_index, options);
    if (split.cost < no_split_cost) {
      split_node(bvh, node_index, split, aabbs, centers, options);
      // NOTE: no fallback split, this is anyways conditional
    }
  }
}

bvh_t build_bvh2(geometry::aabb_t *aabbs, glm::vec3 *centers,
                 uint32_t primitive_count, const options_t &options) {
  bvh_t bvh{};
  bvh.primitive_indices.reserve(primitive_count);
  for (uint32_t i = 0; i < primitive_count; i++)
    bvh.primitive_indices.push_back(i);
  node_t root = new_node(0, primitive_count);
  bvh.nodes.push_back(root);
  update_node_bounds(bvh, 0, aabbs);
  try_split_node(bvh, 0, aabbs, centers, options);
  return bvh;
}

float cost_of_bvh(const bvh_t &bvh, const options_t &options) {
  return cost_of_node(bvh, 0, options);
}

uint32_t depth_of_bvh(const bvh_t &bvh) { return depth_of_node(bvh, 0); }

} // namespace bvh
