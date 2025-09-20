#include "bvh/bvh.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <vector>

#include "math/aabb.hpp"
#include "math/math.hpp"
#include "math/triangle.hpp"

namespace bvh {

uint64_t split(uint64_t x, int log_bits) {
  const int bit_count = 1 << log_bits;
  uint64_t  mask      = ((uint64_t)-1) >> (bit_count / 2);
  x &= mask;
  for (int i = log_bits - 1, n = 1 << i; i > 0; --i, n >>= 1) {
    mask = (mask | (mask << n)) & ~(mask << (n / 2));
    x    = (x | (x << n)) & mask;
  }
  return x;
}

uint64_t encode(uint64_t x, uint64_t y, uint64_t z, int log_bits) {
  return split(x, log_bits) | (split(y, log_bits) << 1) |
         (split(z, log_bits) << 2);
}

uint32_t find_best_node(const std::vector<node_t> &nodes, uint32_t node_index,
                        uint32_t search_radius) {
  const node_t  &node       = nodes[node_index];
  uint32_t       best_index = node_index;
  float          best_area  = math::infinity;
  const uint32_t node_count = nodes.size();
  uint32_t start = node_index > search_radius ? node_index - search_radius : 0;
  uint32_t stop  = node_index + search_radius + 1 < node_count
                       ? node_index + search_radius + 1
                       : node_count;
  for (uint32_t index = start; index < stop; index++) {
    if (index == node_index) continue;
    float area = node.aabb().grow(nodes[index].aabb()).area();
    if (area < best_area) {
      best_area  = area;
      best_index = index;
    }
  }
  return best_index;
}

bvh_t build_bvh(const model::raw_mesh_t &mesh) {
  bvh_t bvh{};

  std::vector<math::aabb_t> aabbs;
  std::vector<math::vec3>   centers;

  for (uint32_t i = 0; i < mesh.indices.size(); i += 3) {
    bvh_triangle_t bvh_triangle{
        {mesh.vertices[mesh.indices[i + 0]].position, 0},
        {mesh.vertices[mesh.indices[i + 1]].position, 0},
        {mesh.vertices[mesh.indices[i + 2]].position, 0},
    };
    bvh.triangles.push_back(bvh_triangle);
    math::aabb_t aabb = bvh_triangle.aabb();
    aabbs.push_back(aabb);
    centers.push_back(aabb.center());
  }

  uint32_t primitive_count = aabbs.size();

  for (uint32_t i = 0; i < primitive_count; i++) {
    bvh.prim_indices.push_back(i);
  }

  math::aabb_t center_aabb{};
  for (uint32_t i = 0; i < centers.size(); i++) {
    center_aabb.grow(centers[i]);
  }

  std::vector<uint32_t> morton_codes(aabbs.size());
  for (uint32_t i = 0; i < aabbs.size(); i++) {
    math::vec3 grid_position = math::min(
        math::vec3{1024 - 1},
        math::max(math::vec3{0},
                  (centers[i] - center_aabb.min) *
                      (math::vec3{1024} /
                       math::vec3{center_aabb.max - center_aabb.min})));
    morton_codes[i] =
        encode(grid_position.x, grid_position.y, grid_position.z, 10);
  }

  std::sort(bvh.prim_indices.begin(), bvh.prim_indices.end(),
            [&](uint32_t a, uint32_t b) {
              return morton_codes[a] < morton_codes[b];
            });

  std::vector<node_t>   current_nodes(primitive_count), next_nodes;
  std::vector<uint32_t> merge_index(primitive_count);

  for (uint32_t i = 0; i < primitive_count; i++) {
    current_nodes[i].prim_count = 1;
    current_nodes[i].index      = i;
    current_nodes[i].min        = aabbs[bvh.prim_indices[i]].min;
    current_nodes[i].max        = aabbs[bvh.prim_indices[i]].max;
  }

  bvh.nodes                = std::vector<node_t>(2 * bvh.triangles.size() - 1);
  uint32_t insertion_index = bvh.nodes.size();

  while (current_nodes.size() > 1) {
    for (uint32_t i = 0; i < current_nodes.size(); i++)
      merge_index[i] = find_best_node(current_nodes, i, 14);
    next_nodes.clear();
    for (size_t i = 0; i < current_nodes.size(); i++) {
      uint32_t j = merge_index[i];
      if (i == merge_index[j]) {
        if (i > j) continue;
        assert(insertion_index >= 2);
        insertion_index -= 2;
        bvh.nodes[insertion_index + 0] = current_nodes[i];
        bvh.nodes[insertion_index + 1] = current_nodes[j];
        math::aabb_t combined_aabb =
            current_nodes[i].aabb().grow(current_nodes[j].aabb());
        node_t parent;
        parent.prim_count = 0;
        parent.index      = insertion_index;
        parent.min        = combined_aabb.min;
        parent.max        = combined_aabb.max;
        next_nodes.push_back(parent);
      } else {
        next_nodes.push_back(current_nodes[i]);
      }
    }
    std::swap(next_nodes, current_nodes);
  }
  assert(insertion_index == 1);

  bvh.nodes[0] = current_nodes[0];

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

float cost_of_node(const bvh_t &bvh, uint32_t node_index) {
  const node_t &node = bvh.nodes[node_index];
  if (node.is_leaf()) return 1.1f * node.prim_count;
  const node_t &left  = bvh.nodes[node.index + 0];
  const node_t &right = bvh.nodes[node.index + 1];
  float         cost  = left.aabb().area() * cost_of_node(bvh, node.index + 0) +
               right.aabb().area() * cost_of_node(bvh, node.index + 1);
  return 1.f + (cost / node.aabb().area());
}

float cost_of_bvh(const bvh_t &bvh) { return cost_of_node(bvh, 0); }

}  // namespace bvh
