#include "bvh/bvh.hpp"

#include <algorithm>
#include <bit>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

#include "glm/common.hpp"
#include "math/aabb.hpp"
#include "math/math.hpp"
#include "math/triangle.hpp"

namespace bvh {

math::vec3 split_edge(const math::vec3 a, const math::vec3 b, uint32_t axis,
                      float position) {
  float t = (position - a[axis]) / (b[axis] - a[axis]);
  return a + t * (b - a);
}

std::pair<math::aabb_t, math::aabb_t> bvh_triangle_t::split(
    uint32_t axis, float position) const {
  math::aabb_t l_aabb{}, r_aabb{};

  bool q0 = v0[axis] <= position;
  bool q1 = v1[axis] <= position;
  bool q2 = v2[axis] <= position;

  if (q0)
    l_aabb.grow(v0);
  else
    r_aabb.grow(v0);
  if (q1)
    l_aabb.grow(v1);
  else
    r_aabb.grow(v1);
  if (q2)
    l_aabb.grow(v2);
  else
    r_aabb.grow(v2);

  if (q0 ^ q1) {
    math::vec3 m = split_edge(v0, v1, axis, position);
    l_aabb.grow(m);
    r_aabb.grow(m);
  }
  if (q1 ^ q2) {
    math::vec3 m = split_edge(v1, v2, axis, position);
    l_aabb.grow(m);
    r_aabb.grow(m);
  }
  if (q2 ^ q0) {
    math::vec3 m = split_edge(v2, v0, axis, position);
    l_aabb.grow(m);
    r_aabb.grow(m);
  }

  return {l_aabb, r_aabb};
}

void update_node_bounds(bvh_t &bvh, uint32_t node_index, math::aabb_t *aabbs) {
  node_t &node = bvh.nodes[node_index];
  if (node.is_leaf()) {
    math::aabb_t aabb{};
    for (uint32_t i = 0; i < node.prim_count; i++)
      aabb.grow(aabbs[bvh.prim_indices[node.index + i]]);
    node.min = aabb.min;
    node.max = aabb.max;
  } else {
    assert(false && "shouldnt reach here");
  }
}

float cost_of_node(const bvh_t &bvh, uint32_t node_index) {
  const node_t &node = bvh.nodes[node_index];
  if (node.is_leaf()) return 1.1f * node.prim_count;
  const node_t &left  = bvh.nodes[node.index + 0];
  const node_t &right = bvh.nodes[node.index + 1];
  float         cost  = left.aabb().area() * cost_of_node(bvh, node.index + 0) +
               right.aabb().area() * cost_of_node(bvh, node.index + 1);
  return 1.f + (cost / node.aabb().area());
}

float greedy_cost_of_node(uint32_t left_count, uint32_t right_count,
                          float left_aabb_area, float right_aabb_area,
                          float parent_aabb_area) {
  return 1.f + ((left_aabb_area * 1.1f * left_count +
                 right_aabb_area * 1.1f * right_count) /
                parent_aabb_area);
}

struct split_t {
  uint32_t axis     = std::numeric_limits<uint32_t>::max();
  float    position = math::infinity;
  float    cost     = math::infinity;
};

struct bin_t {
  math::aabb_t aabb{};
  uint32_t     prim_count = 0;
};

split_t find_best_object_split(bvh_t &bvh, uint32_t node_index,
                               math::aabb_t *aabbs, math::vec3 *centers) {
  const uint32_t num_samples = 8;

  node_t      &node = bvh.nodes[node_index];
  split_t      best_split{};
  math::aabb_t split_bounds{};
  for (uint32_t i = 0; i < node.prim_count; i++)
    split_bounds.grow(centers[bvh.prim_indices[node.index + i]]);

  for (uint32_t axis = 0; axis < 3; axis++) {
    if (split_bounds.max[axis] == split_bounds.min[axis]) continue;

    bin_t bins[num_samples];
    for (auto &bin : bins) bin = {.aabb{math::vec3{0}, math::vec3{0}}};

    float scale = static_cast<float>(num_samples) /
                  (split_bounds.max[axis] - split_bounds.min[axis]);
    for (uint32_t i = 0; i < node.prim_count; i++) {
      uint32_t bin_index =
          std::min(num_samples - 1,
                   static_cast<uint32_t>(
                       (centers[bvh.prim_indices[node.index + i]][axis] -
                        split_bounds.min[axis]) *
                       scale));
      bins[bin_index].prim_count++;
      bins[bin_index].aabb.grow(aabbs[bvh.prim_indices[node.index + i]]);
    }

    float        left_area[num_samples - 1], right_area[num_samples - 1];
    uint32_t     left_count[num_samples - 1], right_count[num_samples - 1];
    math::aabb_t left_aabb{}, right_aabb{};
    uint32_t     left_sum = 0, right_sum = 0;
    for (uint32_t i = 0; i < num_samples - 1; i++) {
      left_sum += bins[i].prim_count;
      left_count[i] = left_sum;
      left_aabb.grow(bins[i].aabb);
      left_area[i] = left_aabb.area();

      right_sum += bins[num_samples - i - 1].prim_count;
      right_count[num_samples - i - 2] = right_sum;
      right_aabb.grow(bins[num_samples - i - 1].aabb);
      right_area[num_samples - i - 2] = right_aabb.area();
    }

    scale = (split_bounds.max[axis] - split_bounds.min[axis]) /
            static_cast<float>(num_samples);
    for (uint32_t i = 0; i < num_samples - 1; i++) {
      float cost =
          greedy_cost_of_node(left_count[i], right_count[i], left_area[i],
                              right_area[i], node.aabb().area());
      if (cost < best_split.cost) {
        best_split.cost     = cost;
        best_split.axis     = axis;
        best_split.position = split_bounds.min[axis] + scale * (i + 1);
      }
    }
  }

  return best_split;
}

uint32_t partition_primitive_indices(bvh_t &bvh, uint32_t node_index,
                                     split_t split, math::vec3 *centers) {
  node_t &node = bvh.nodes[node_index];
  auto    middle =
      std::partition(bvh.prim_indices.begin() + node.index,
                     bvh.prim_indices.begin() + node.index + node.prim_count,
                     [&](uint32_t index) {
                       return centers[index][split.axis] < split.position;
                     });
  return std::distance(bvh.prim_indices.begin(), middle);
}

void try_split_node(bvh_t &bvh, uint32_t node_index, math::aabb_t *aabbs,
                    math::vec3 *centers);

void split_node(bvh_t &bvh, uint32_t node_index, split_t split,
                math::aabb_t *aabbs, math::vec3 *centers) {
  if (split.axis == std::numeric_limits<uint32_t>::max()) return;
  node_t &node = bvh.nodes[node_index];

  uint32_t right_index =
      partition_primitive_indices(bvh, node_index, split, centers);

  if (node.index == right_index || node.index + node.prim_count == right_index)
    return;

  node_t left, right;

  left.index      = node.index;
  left.prim_count = right_index - node.index;

  right.index      = right_index;
  right.prim_count = node.index + node.prim_count - right_index;

  node.prim_count = 0;
  node.index      = bvh.nodes.size();

  uint32_t left_node_index = bvh.nodes.size();
  bvh.nodes.push_back(left);

  uint32_t right_node_index = bvh.nodes.size();
  bvh.nodes.push_back(right);

  update_node_bounds(bvh, left_node_index, aabbs);
  update_node_bounds(bvh, right_node_index, aabbs);

  try_split_node(bvh, left_node_index, aabbs, centers);
  try_split_node(bvh, right_node_index, aabbs, centers);
}

void try_split_node(bvh_t &bvh, uint32_t node_index, math::aabb_t *aabbs,
                    math::vec3 *centers) {
  node_t &node = bvh.nodes[node_index];
  if (node.prim_count <= 1) return;

  if (node.prim_count > 1) {
    const split_t split =
        find_best_object_split(bvh, node_index, aabbs, centers);
    split_node(bvh, node_index, split, aabbs, centers);
  } else {
    float   no_split_cost = cost_of_node(bvh, node_index);
    split_t split = find_best_object_split(bvh, node_index, aabbs, centers);
    if (split.cost < no_split_cost) {
      split_node(bvh, node_index, split, aabbs, centers);
    }
  }
}

// TODO: clean dead nodes
void collapse_nodes(bvh_t &bvh, uint32_t node_index) {
  node_t &node = bvh.nodes[node_index];
  if (node.is_leaf()) return;
  collapse_nodes(bvh, node.index + 0);
  collapse_nodes(bvh, node.index + 1);
  node_t &left  = bvh.nodes[node.index + 0];
  node_t &right = bvh.nodes[node.index + 1];
  if (left.is_leaf() && right.is_leaf()) {
    float real_cost    = cost_of_node(bvh, node_index);
    float cost_if_leaf = 1.1f * (left.prim_count + right.prim_count);
    if (cost_if_leaf <= real_cost) {
      assert(right.index == left.index + left.prim_count);
      node.prim_count  = left.prim_count + right.prim_count;
      node.index       = left.index;
      left.prim_count  = 0;
      right.prim_count = 0;
    }
  }
}

float priority(const math::aabb_t aabb, const bvh_triangle_t triangle) {
  return std::cbrt(std::pow(aabb.largest_extent(), 2.f) * aabb.area() *
                   triangle.area());
}

uint32_t num_splits(float total_priority, const bvh_triangle_t triangle,
                    uint32_t triangle_count) {
  float share =
      priority(triangle.aabb(), triangle) / total_priority * triangle_count;
  return 1 + (uint32_t)(share * 0.3);  // split factor
}

float get_cell_size(float alpha) {
  uint32_t float_bits = std::bit_cast<uint32_t>(alpha);
  float_bits &= 255u << 23;
  return std::bit_cast<float>(float_bits);
}

std::pair<std::vector<math::aabb_t>, std::vector<uint32_t>> presplit(
    const std::vector<bvh_triangle_t> &triangles) {
  math::aabb_t global_aabb{};
  for (const auto &triangle : triangles) global_aabb.grow(triangle.aabb());
  math::vec3 global_extent = global_aabb.extent();

  float total_priority = 0;
  for (const auto &triangle : triangles)
    total_priority += priority(triangle.aabb(), triangle);

  uint32_t total_splits = 0;
  for (const auto &triangle : triangles)
    total_splits += num_splits(total_priority, triangle, triangles.size());

  std::vector<math::aabb_t> aabbs(total_splits);
  std::vector<uint32_t>     tri_indices(total_splits);

  std::pair<math::aabb_t, uint32_t> stack[64];

  uint32_t split_index = 0;

  for (uint32_t i = 0; i < triangles.size(); i++) {
    const bvh_triangle_t &triangle = triangles[i];

    uint32_t split_count =
        num_splits(total_priority, triangle, triangles.size());

    uint32_t stack_top = 0;
    stack[stack_top++] = {triangle.aabb(), split_count};

    while (stack_top) {
      auto [aabb, splits_left] = stack[--stack_top];
      if (splits_left == 1) {
        aabbs[split_index]       = aabb;
        tri_indices[split_index] = i;
        split_index++;
        continue;
      }

      uint32_t split_axis     = aabb.largest_axis();
      float    largest_extent = aabb.largest_extent();

      float alpha     = largest_extent / global_extent[split_axis];
      float cell_size = get_cell_size(alpha) * global_extent[split_axis];
      if (cell_size >= largest_extent - 0.0001f) {
        cell_size *= 0.5f;
      }

      float mid_pos = aabb.center()[split_axis];

      float split_pos =
          global_aabb.min[split_axis] +
          math::round((mid_pos - global_aabb.min[split_axis]) / cell_size) *
              cell_size;
      if (split_pos < aabb.min[split_axis] ||
          split_pos > aabb.max[split_axis]) {
        split_pos = mid_pos;
      }

      auto [l_aabb, r_aabb] = triangle.split(split_axis, split_pos);
      l_aabb.shrink(aabb);
      r_aabb.shrink(aabb);

      float left_extent  = l_aabb.largest_extent();
      float right_extent = r_aabb.largest_extent();

      uint32_t left_count = math::round(
          splits_left * (left_extent / (left_extent + right_extent)));
      left_count = math::clamp(left_count, 1u, splits_left - 1);

      uint32_t right_count = splits_left - left_count;

      stack[stack_top++] = {r_aabb, right_count};
      stack[stack_top++] = {l_aabb, left_count};
    }
  }

  return {aabbs, tri_indices};
}

bvh_t build_bvh(const model::raw_mesh_t &mesh) {
  bvh_t bvh{};

  for (uint32_t i = 0; i < mesh.indices.size(); i += 3) {
    bvh_triangle_t bvh_triangle{
        {mesh.vertices[mesh.indices[i + 0]].position, 0},
        {mesh.vertices[mesh.indices[i + 1]].position, 0},
        {mesh.vertices[mesh.indices[i + 2]].position, 0},
    };
    bvh.triangles.push_back(bvh_triangle);
  }

  auto [aabbs, tri_indices] = presplit(bvh.triangles);

  std::vector<math::vec3> centers;

  for (const auto &aabb : aabbs) centers.push_back(aabb.center());

  uint32_t primitive_count = aabbs.size();

  for (uint32_t i = 0; i < primitive_count; i++) bvh.prim_indices.push_back(i);

  node_t root;
  root.index      = 0;
  root.prim_count = primitive_count;
  bvh.nodes.push_back(root);

  update_node_bounds(bvh, 0, aabbs.data());
  try_split_node(bvh, 0, aabbs.data(), centers.data());

  collapse_nodes(bvh, 0);

  for (auto &node : bvh.nodes) {
    if (!node.is_leaf()) continue;
    for (uint32_t i = 0; i < node.prim_count; i++) {
      uint32_t index                   = bvh.prim_indices[node.index + i];
      uint32_t tri_index               = tri_indices[index];
      bvh.prim_indices[node.index + i] = tri_index;
    }
  }

  return bvh;
}

uint32_t depth_of_node(const bvh_t &bvh, uint32_t node_id) {
  const node_t &node = bvh.nodes[node_id];
  if (node.is_leaf()) return 0;
  return std::max(depth_of_node(bvh, node.index + 0),
                  depth_of_node(bvh, node.index + 1)) +
         1;
}

uint32_t depth_of_bvh(const bvh_t &bvh) { return depth_of_node(bvh, 0); }

float cost_of_bvh(const bvh_t &bvh) { return cost_of_node(bvh, 0); }

}  // namespace bvh
