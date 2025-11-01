#include "bvh/bvh.hpp"

#include <algorithm>
#include <bit>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "glm/common.hpp"
#include "math/aabb.hpp"
#include "math/math.hpp"
#include "math/triangle.hpp"
#include "model/model.hpp"

namespace bvh {

static const float triangle_intersection_cost = 1.1f;
static const float node_traversal_cost        = 1.f;

void update_node_bounds(bvh_t &bvh, const std::vector<uint32_t> &prim_indices,
                        uint32_t node_index, const math::aabb_t *aabbs) {
  node_t &node = bvh.nodes[node_index];
  if (node.is_leaf()) {
    math::aabb_t aabb{};
    for (uint32_t i = 0; i < node.prim_count; i++)
      aabb.grow(aabbs[prim_indices[node.index + i]]);
    node.min = aabb.min;
    node.max = aabb.max;
  } else {
    assert(false && "shouldnt reach here");
  }
}

float sah_of_node(const bvh_t &bvh, uint32_t node_index) {
  const node_t &node = bvh.nodes[node_index];
  if (node.is_leaf()) return triangle_intersection_cost * node.prim_count;
  const node_t &left        = bvh.nodes[node.index + 0];
  const node_t &right       = bvh.nodes[node.index + 1];
  float         left_cost   = sah_of_node(bvh, node.index + 0);
  float         right_cost  = sah_of_node(bvh, node.index + 1);
  float         left_area   = left.aabb().area();
  float         right_area  = right.aabb().area();
  float         parent_area = node.aabb().area();
  return node_traversal_cost + (left_area / parent_area) * left_cost +
         (right_area / parent_area) * right_cost;
}

uint32_t depth_of_node(const bvh_t &bvh, uint32_t node_id) {
  const node_t &node = bvh.nodes[node_id];
  if (node.is_leaf()) return 0;
  return std::max(depth_of_node(bvh, node.index + 0),
                  depth_of_node(bvh, node.index + 1)) +
         1;
}

uint32_t depth_of_bvh(const bvh_t &bvh) { return depth_of_node(bvh, 0); }

float sah_of_bvh(const bvh_t &bvh) {
  float cost = 0;
  for (const auto &node : bvh.nodes) {
    if (node.is_leaf()) {
      cost += triangle_intersection_cost * node.prim_count * node.aabb().area();
    } else {
      cost += node_traversal_cost * node.aabb().area();
    }
  }
  return cost / bvh.nodes[0].aabb().area();
}

math::vec3 intersect(math::vec3 p1, math::vec3 p2, math::vec3 plane_p,
                     math::vec3 plane_n) {
  math::vec3 line_dir = p2 - p1;
  float      dot_line_normal =
      plane_n.x * line_dir.x + plane_n.y * line_dir.y + plane_n.z * line_dir.z;
  if (std::abs(dot_line_normal) < 1e-6) return p1;

  math::vec3 w            = p1 - plane_p;
  float      dot_w_normal = plane_n.x * w.x + plane_n.y * w.y + plane_n.z * w.z;
  float      t            = -dot_w_normal / dot_line_normal;

  return p1 + line_dir * t;
}

void clip_polygon_plane(std::vector<math::vec3> &polygon, math::vec3 plane_p,
                        math::vec3 plane_n) {
  std::vector<math::vec3> new_polygon;
  if (polygon.empty()) return;

  for (size_t i = 0; i < polygon.size(); ++i) {
    math::vec3 p1 = polygon[i];
    math::vec3 p2 = polygon[(i + 1) % polygon.size()];

    float p1_pos = (p1.x - plane_p.x) * plane_n.x +
                   (p1.y - plane_p.y) * plane_n.y +
                   (p1.z - plane_p.z) * plane_n.z;
    float p2_pos = (p2.x - plane_p.x) * plane_n.x +
                   (p2.y - plane_p.y) * plane_n.y +
                   (p2.z - plane_p.z) * plane_n.z;

    if (p1_pos <= 0) {
      if (p2_pos <= 0) {
        new_polygon.push_back(p2);
      } else {
        new_polygon.push_back(intersect(p1, p2, plane_p, plane_n));
      }
    } else {
      if (p2_pos <= 0) {
        new_polygon.push_back(intersect(p1, p2, plane_p, plane_n));
        new_polygon.push_back(p2);
      }
    }
  }
  polygon = new_polygon;
}

std::vector<math::vec3> clip_triangle(const math::triangle_t &triangle,
                                      const math::aabb_t     &aabb) {
  std::vector<math::vec3> polygon = {triangle.v0, triangle.v1, triangle.v2};

  std::vector<std::pair<math::vec3, math::vec3>> planes = {
      {aabb.min, {-1, 0, 0}}, {aabb.max, {1, 0, 0}},  {aabb.min, {0, -1, 0}},
      {aabb.max, {0, 1, 0}},  {aabb.min, {0, 0, -1}}, {aabb.max, {0, 0, 1}}};

  for (const auto &plane : planes) {
    clip_polygon_plane(polygon, plane.first, plane.second);
  }

  return polygon;
}

float polygon_area(const std::vector<math::vec3> &polygon) {
  if (polygon.size() < 3) return 0.0f;

  math::vec3 total = {0, 0, 0};
  for (size_t i = 0; i < polygon.size(); ++i) {
    math::vec3 p1 = polygon[i];
    math::vec3 p2 = polygon[(i + 1) % polygon.size()];
    total.x += (p1.y - p2.y) * (p1.z + p2.z);
    total.y += (p1.z - p2.z) * (p1.x + p2.x);
    total.z += (p1.x - p2.x) * (p1.y + p2.y);
  }

  return 0.5f *
         std::sqrt(total.x * total.x + total.y * total.y + total.z * total.z);
}

std::set<uint32_t> get_primitives_intersecting_node(
    const bvh_t &bvh, const math::aabb_t &aabb,
    const std::vector<math::triangle_t> &triangles) {
  uint32_t stack[64], stack_top = 0;
  stack[stack_top++] = 0;
  std::set<uint32_t> intersections{};
  while (stack_top) {
    const node_t &node = bvh.nodes[stack[--stack_top]];
    if (node.aabb().intersects(aabb)) {
      if (node.is_leaf()) {
        for (uint32_t i = 0; i < node.prim_count; i++) {
          if (aabb.intersects(
                  triangles[bvh.prim_indices[node.index + i]].aabb()))
            intersections.emplace(bvh.prim_indices[node.index + i]);
        }
      } else {
        stack[stack_top++] = node.index + 1;
        stack[stack_top++] = node.index + 0;
      }
    }
  }
  return intersections;
}

std::set<uint32_t> get_primitives_in_node(const bvh_t &bvh,
                                          uint32_t     node_index) {
  std::set<uint32_t> indices;
  uint32_t           stack[64], stack_top = 0;
  stack[stack_top++] = node_index;
  while (stack_top) {
    const node_t &node = bvh.nodes[stack[--stack_top]];
    if (node.is_leaf()) {
      for (uint32_t i = 0; i < node.prim_count; i++) {
        indices.emplace(bvh.prim_indices[node.index + i]);
      }
    } else {
      stack[stack_top++] = node.index + 1;
      stack[stack_top++] = node.index + 0;
    }
  }
  return indices;
}

float epo_of_bvh(const bvh_t                         &bvh,
                 const std::vector<math::triangle_t> &triangles) {
  float total_triangle_area = 0;
  for (const auto &tri : triangles) total_triangle_area += tri.area();

  const uint32_t num_threads = 16;
  float          epo_per_thread[num_threads];
  for (auto &epo : epo_per_thread) epo = 0;

  std::vector<std::thread> threads{};
  for (uint32_t thread_index = 0; thread_index < num_threads; thread_index++) {
    threads.emplace_back(
        [&](uint32_t thread_index) {
          float &epo = epo_per_thread[thread_index];
          for (uint32_t current_node_index = thread_index + 1;
               current_node_index < bvh.nodes.size();
               current_node_index += num_threads) {
            const node_t &current = bvh.nodes[current_node_index];
            // TODO: optimise
            std::set<uint32_t> all_triangles_intersecting_node =
                get_primitives_intersecting_node(bvh, current.aabb(),
                                                 triangles);
            std::set<uint32_t> all_triangles_in_node =
                get_primitives_in_node(bvh, current_node_index);

            std::set<uint32_t>
                all_triangles_intersecting_node_but_not_in_node{};
            for (auto index : all_triangles_intersecting_node)
              if (!all_triangles_in_node.contains(index))
                all_triangles_intersecting_node_but_not_in_node.emplace(index);

            float overlap = 0;
            for (auto tri_index :
                 all_triangles_intersecting_node_but_not_in_node) {
              overlap += polygon_area(
                  clip_triangle(triangles[tri_index], current.aabb()));
            }

            if (current.is_leaf()) {
              epo += triangle_intersection_cost * current.prim_count * overlap /
                     total_triangle_area;
            } else {
              epo += node_traversal_cost * overlap / total_triangle_area;
            }
          }
        },
        thread_index);
  }
  for (auto &thread : threads) thread.join();

  float epo = 0;
  for (uint32_t i = 0; i < num_threads; i++) epo += epo_per_thread[i];

  return epo;
}

float greedy_sah_of_node(uint32_t left_count, uint32_t right_count,
                         float left_aabb_area, float right_aabb_area,
                         float parent_aabb_area) {
  return node_traversal_cost +
         ((left_aabb_area * triangle_intersection_cost * left_count +
           right_aabb_area * triangle_intersection_cost * right_count) /
          parent_aabb_area);
}

struct split_t {
  uint32_t axis     = std::numeric_limits<uint32_t>::max();
  float    position = math::infinity;
  float    cost     = math::infinity;
  uint32_t index    = std::numeric_limits<uint32_t>::max();
};

struct bin_t {
  math::aabb_t aabb{};
  uint32_t     prim_count = 0;
};

split_t find_best_object_split_binned_sah(bvh_t &bvh, uint32_t node_index,
                                          const math::aabb_t *aabbs,
                                          const math::vec3   *centers,
                                          const uint32_t      num_samples) {
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
          greedy_sah_of_node(left_count[i], right_count[i], left_area[i],
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

split_t find_best_object_split_sweep_sah(
    bvh_t &bvh, uint32_t node_index, const math::aabb_t *aabbs,
    const math::vec3           *centers,
    const std::vector<uint32_t> sorted_prim_indices[3],
    std::vector<float>         &right_costs) {
  node_t &node = bvh.nodes[node_index];

  float parent_area = node.aabb().area();

  split_t best_split{};

  for (uint32_t axis = 0; axis < 3; axis++) {
    math::aabb_t right_aabb{};
    uint32_t     right_counter = 0;
    for (int32_t index = node.index + node.prim_count - 1;
         index >= node.index + 1; index--) {
      right_counter++;
      right_aabb.grow(aabbs[sorted_prim_indices[axis][index]]);
      float right_cost   = right_aabb.area() * right_counter;
      right_costs[index] = right_cost;
      // TODO: add early out
    }
    math::aabb_t left_aabb{};
    uint32_t     left_counter = 0;
    for (uint32_t index = node.index; index < node.prim_count + node.index - 1;
         index++) {
      left_counter++;
      left_aabb.grow(aabbs[sorted_prim_indices[axis][index]]);
      float left_cost  = left_aabb.area() * left_counter;
      float right_cost = right_costs[index + 1];
      float cost =
          node_traversal_cost +
          (triangle_intersection_cost * (left_cost + right_cost) / parent_area);

      if (cost < best_split.cost) {
        best_split.cost = cost;
        best_split.axis = axis;
        best_split.position =
            centers[sorted_prim_indices[axis][index + 1]][axis];
        best_split.index = index + 1;
      }
    }
  }

  return best_split;
}

uint32_t partition_primitive_indices_on_split_position(
    bvh_t &bvh, std::vector<uint32_t> &prim_indices, uint32_t node_index,
    split_t split, const math::vec3 *centers) {
  node_t &node   = bvh.nodes[node_index];
  auto    middle = std::stable_partition(
      prim_indices.begin() + node.index,
      prim_indices.begin() + node.index + node.prim_count, [&](uint32_t index) {
        return centers[index][split.axis] < split.position;
      });
  return std::distance(prim_indices.begin(), middle);
}

uint32_t partition_primitive_indices_on_split_index(
    bvh_t &bvh, std::vector<uint32_t> &prim_indices, uint32_t node_index,
    split_t split, const math::vec3 *centers) {
  node_t &node   = bvh.nodes[node_index];
  auto    middle = std::stable_partition(
      prim_indices.begin() + node.index,
      prim_indices.begin() + node.index + node.prim_count,
      [&](uint32_t index) { return index < split.index; });
  return std::distance(prim_indices.begin(), middle);
}

void try_split_node_binned_sah(bvh_t &bvh, uint32_t node_index,
                               const math::aabb_t *aabbs,
                               const math::vec3 *centers, uint32_t num_samples,
                               uint32_t min_primitives,
                               uint32_t max_primitives);

void try_split_node_sweep_sah(bvh_t &bvh, uint32_t node_index,
                              const math::aabb_t    *aabbs,
                              const math::vec3      *centers,
                              std::vector<uint32_t> *sorted_prim_indices,
                              std::vector<bool>     &move_left,
                              std::vector<float>    &right_costs,
                              uint32_t min_primitives, uint32_t max_primitives);

void split_node_binned_sah(bvh_t &bvh, uint32_t node_index, split_t split,
                           const math::aabb_t *aabbs, const math::vec3 *centers,
                           uint32_t num_samples, uint32_t min_primitives,
                           uint32_t max_primitives) {
  if (split.axis == std::numeric_limits<uint32_t>::max()) return;
  node_t &node = bvh.nodes[node_index];

  uint32_t right_index = partition_primitive_indices_on_split_position(
      bvh, bvh.prim_indices, node_index, split, centers);

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

  update_node_bounds(bvh, bvh.prim_indices, left_node_index, aabbs);
  update_node_bounds(bvh, bvh.prim_indices, right_node_index, aabbs);

  try_split_node_binned_sah(bvh, left_node_index, aabbs, centers, num_samples,
                            min_primitives, max_primitives);
  try_split_node_binned_sah(bvh, right_node_index, aabbs, centers, num_samples,
                            min_primitives, max_primitives);
}

void split_node_sweep_sah(bvh_t &bvh, uint32_t node_index, split_t split,
                          const math::aabb_t *aabbs, const math::vec3 *centers,
                          std::vector<uint32_t> *sorted_prim_indices,
                          std::vector<bool>     &move_left,
                          std::vector<float>    &right_costs,
                          uint32_t min_primitives, uint32_t max_primitives) {
  if (split.axis == std::numeric_limits<uint32_t>::max()) return;
  node_t &node = bvh.nodes[node_index];

  auto    &sorted_prim_indices_split_axis = sorted_prim_indices[split.axis];
  uint32_t right_index                    = split.index;
  for (uint32_t i = node.index; i < right_index; i++) {
    move_left[sorted_prim_indices_split_axis[i]] = true;
  }
  for (uint32_t i = right_index; i < node.index + node.prim_count; i++) {
    move_left[sorted_prim_indices_split_axis[i]] = false;
  }
  for (uint32_t axis = 0; axis < 3; axis++) {
    if (split.axis == axis) continue;
    auto &sorted_prim_indices_axis = sorted_prim_indices[axis];
    std::stable_partition(
        sorted_prim_indices_axis.begin() + node.index,
        sorted_prim_indices_axis.begin() + node.index + node.prim_count,
        [&](uint32_t index) { return move_left[index]; });
  }

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

  update_node_bounds(bvh, sorted_prim_indices[split.axis], left_node_index,
                     aabbs);
  update_node_bounds(bvh, sorted_prim_indices[split.axis], right_node_index,
                     aabbs);

  try_split_node_sweep_sah(bvh, left_node_index, aabbs, centers,
                           sorted_prim_indices, move_left, right_costs,
                           min_primitives, max_primitives);
  try_split_node_sweep_sah(bvh, right_node_index, aabbs, centers,
                           sorted_prim_indices, move_left, right_costs,
                           min_primitives, max_primitives);
}

void try_split_node_binned_sah(bvh_t &bvh, uint32_t node_index,
                               const math::aabb_t *aabbs,
                               const math::vec3 *centers, uint32_t num_samples,
                               uint32_t min_primitives,
                               uint32_t max_primitives) {
  node_t &node = bvh.nodes[node_index];
  if (node.prim_count <= min_primitives) return;

  if (node.prim_count > max_primitives) {
    const split_t split = find_best_object_split_binned_sah(
        bvh, node_index, aabbs, centers, num_samples);
    split_node_binned_sah(bvh, node_index, split, aabbs, centers, num_samples,
                          min_primitives, max_primitives);
  } else {
    float   no_split_cost = sah_of_node(bvh, node_index);
    split_t split = find_best_object_split_binned_sah(bvh, node_index, aabbs,
                                                      centers, num_samples);
    if (split.cost < no_split_cost) {
      split_node_binned_sah(bvh, node_index, split, aabbs, centers, num_samples,
                            min_primitives, max_primitives);
    }
  }
}

void try_split_node_sweep_sah(
    bvh_t &bvh, uint32_t node_index, const math::aabb_t *aabbs,
    const math::vec3 *centers, std::vector<uint32_t> *sorted_prim_indices,
    std::vector<bool> &move_left, std::vector<float> &right_costs,
    uint32_t min_primitives, uint32_t max_primitives) {
  node_t &node = bvh.nodes[node_index];
  if (node.prim_count <= min_primitives) return;

  if (node.prim_count > max_primitives) {
    const split_t split = find_best_object_split_sweep_sah(
        bvh, node_index, aabbs, centers, sorted_prim_indices, right_costs);
    split_node_sweep_sah(bvh, node_index, split, aabbs, centers,
                         sorted_prim_indices, move_left, right_costs,
                         min_primitives, max_primitives);
  } else {
    float   no_split_cost = sah_of_node(bvh, node_index);
    split_t split         = find_best_object_split_sweep_sah(
        bvh, node_index, aabbs, centers, sorted_prim_indices, right_costs);
    if (split.cost < no_split_cost) {
      split_node_sweep_sah(bvh, node_index, split, aabbs, centers,
                           sorted_prim_indices, move_left, right_costs,
                           min_primitives, max_primitives);
    }
  }
}

void collapse_nodes(bvh_t &bvh, uint32_t node_index,
                    std::set<uint32_t> &deadnodes) {
  node_t &node = bvh.nodes[node_index];
  if (node.is_leaf()) return;
  collapse_nodes(bvh, node.index + 0, deadnodes);
  collapse_nodes(bvh, node.index + 1, deadnodes);
  uint32_t left_index  = node.index + 0;
  uint32_t right_index = node.index + 1;
  node_t  &left        = bvh.nodes[left_index];
  node_t  &right       = bvh.nodes[right_index];
  if (left.is_leaf() && right.is_leaf()) {
    float real_cost = sah_of_node(bvh, node_index);
    float cost_if_leaf =
        triangle_intersection_cost * (left.prim_count + right.prim_count);
    if (cost_if_leaf <= real_cost) {
      assert(right.index == left.index + left.prim_count);
      deadnodes.emplace(left_index);
      deadnodes.emplace(right_index);
      node.prim_count  = left.prim_count + right.prim_count;
      node.index       = left.index;
      left.prim_count  = 0;
      right.prim_count = 0;
    }
  }
}

void remove_deadnodes(bvh_t &bvh, const std::set<uint32_t> &deadnodes) {
  std::vector<node_t>   nodes;
  std::vector<uint32_t> old_to_new_index(bvh.nodes.size());
  std::vector<uint32_t> new_to_old;
  for (uint32_t i = 0; i < bvh.nodes.size(); i++) {
    if (!deadnodes.contains(i)) {
      uint32_t new_node_index = nodes.size();
      old_to_new_index[i]     = new_node_index;
      nodes.push_back(bvh.nodes[i]);
      new_to_old.push_back(i);
    }
  }
  for (uint32_t i = 0; i < nodes.size(); i++) {
    if (!nodes[i].is_leaf()) {
      nodes[i].index = old_to_new_index[nodes[i].index];
    }
  }
  bvh.nodes = nodes;
}

float priority(const math::triangle_t triangle) {
  const math::aabb_t aabb = triangle.aabb();
  return std::cbrt(std::pow(aabb.largest_extent(), 2.f) *
                   (aabb.area() - triangle.area()));
}

uint32_t num_splits(float total_priority, const math::triangle_t triangle,
                    uint32_t triangle_count, float split_factor) {
  return 1 + (uint32_t)(priority(triangle) *
                        (triangle_count * split_factor / total_priority));
}

std::pair<std::vector<math::aabb_t>, std::vector<uint32_t>> presplit(
    const std::vector<math::triangle_t> &triangles, float split_factor) {
  math::aabb_t global_aabb{};
  for (const auto &triangle : triangles) global_aabb.grow(triangle.aabb());
  math::vec3 global_extent = global_aabb.extent();

  float total_priority = 0;
  for (const auto &triangle : triangles) total_priority += priority(triangle);

  uint32_t total_splits = 0;
  for (const auto &triangle : triangles)
    total_splits +=
        num_splits(total_priority, triangle, triangles.size(), split_factor);

  std::cout << "total_priority: " << total_priority << '\n';
  std::cout << "total_splits: " << total_splits << '\n';
  std::cout << "total_tris: " << triangles.size() << '\n';

  std::vector<math::aabb_t> aabbs(total_splits);
  std::vector<uint32_t>     tri_indices(total_splits);

  std::pair<math::aabb_t, uint32_t> stack[64];

  uint32_t split_index = 0;

  for (uint32_t i = 0; i < triangles.size(); i++) {
    const math::triangle_t &triangle = triangles[i];

    uint32_t split_count =
        num_splits(total_priority, triangle, triangles.size(), split_factor);

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

      float depth =
          std::min(-1.f, std::floor(std::log2(aabb.largest_extent() /
                                              global_extent[split_axis])));
      float cell_size = std::exp2(depth) * global_extent[split_axis];
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

      uint32_t left_count = (uint32_t)math::round(
          splits_left * (left_extent / (left_extent + right_extent)));
      left_count = math::clamp(left_count, 1u, splits_left - 1);

      uint32_t right_count = splits_left - left_count;

      stack[stack_top++] = {r_aabb, right_count};
      stack[stack_top++] = {l_aabb, left_count};
    }
  }

  return {aabbs, tri_indices};
}

void presplit_remove_indirection(bvh_t                       &bvh,
                                 const std::vector<uint32_t> &tri_indices) {
  for (auto &node : bvh.nodes) {
    if (!node.is_leaf()) continue;
    for (uint32_t i = 0; i < node.prim_count; i++) {
      uint32_t index                   = bvh.prim_indices[node.index + i];
      uint32_t tri_index               = tri_indices[index];
      bvh.prim_indices[node.index + i] = tri_index;
    }
  }
}

void presplit_remove_duplicates(bvh_t &bvh) {
  std::vector<uint32_t> prim_indices{};
  uint32_t              stack[64], stack_top = 0;
  stack[stack_top++] = 0;

  while (stack_top) {
    node_t &node = bvh.nodes[stack[--stack_top]];
    if (node.is_leaf()) {
      std::set<uint32_t> node_prim_indices;
      for (uint32_t i = 0; i < node.prim_count; i++) {
        uint32_t index = bvh.prim_indices[node.index + i];
        node_prim_indices.emplace(index);
      }
      node.index = prim_indices.size();
      for (auto index : node_prim_indices) {
        prim_indices.push_back(index);
      }
      node.prim_count = node_prim_indices.size();
    } else {
      stack[stack_top++] = node.index + 1;
      stack[stack_top++] = node.index + 0;
    }
  }
  bvh.prim_indices = prim_indices;
}

uint64_t split_morton(uint64_t x, int log_bits) {
  const int bit_count = 1 << log_bits;
  uint64_t  mask      = ((uint64_t)-1) >> (bit_count / 2);
  x &= mask;
  for (int i = log_bits - 1, n = 1 << i; i > 0; --i, n >>= 1) {
    mask = (mask | (mask << n)) & ~(mask << (n / 2));
    x    = (x | (x << n)) & mask;
  }
  return x;
}

uint64_t encode_morton(uint64_t x, uint64_t y, uint64_t z, int log_bits) {
  return split_morton(x, log_bits) | (split_morton(y, log_bits) << 1) |
         (split_morton(z, log_bits) << 2);
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

void reorder_indices(bvh_t &bvh, std::vector<uint32_t> &reordered_indices,
                     uint32_t node_index) {
  node_t &node = bvh.nodes[node_index];
  if (node.is_leaf()) {
    uint32_t start_index = reordered_indices.size();
    for (uint32_t i = 0; i < node.prim_count; i++) {
      reordered_indices.push_back(bvh.prim_indices[node.index + i]);
    }
    node.index = start_index;
    return;
  }
  reorder_indices(bvh, reordered_indices, node.index + 0);
  reorder_indices(bvh, reordered_indices, node.index + 1);
}

void fix_primitive_indices(bvh_t &bvh) {
  std::vector<uint32_t> reordered_indices;
  reordered_indices.reserve(bvh.prim_indices.size());
  reorder_indices(bvh, reordered_indices, 0);
  bvh.prim_indices = reordered_indices;
}

bvh_t build_bvh_binned_sah(const std::vector<math::aabb_t> &aabbs,
                           uint32_t num_samples, uint32_t min_primitives,
                           uint32_t max_primitives) {
  bvh_t bvh{};

  std::vector<math::vec3> centers;
  for (const auto &aabb : aabbs) centers.push_back(aabb.center());

  uint32_t primitive_count = aabbs.size();

  for (uint32_t i = 0; i < primitive_count; i++) bvh.prim_indices.push_back(i);

  node_t root;
  root.index      = 0;
  root.prim_count = primitive_count;
  bvh.nodes.push_back(root);

  update_node_bounds(bvh, bvh.prim_indices, 0, aabbs.data());
  try_split_node_binned_sah(bvh, 0, aabbs.data(), centers.data(), num_samples,
                            min_primitives, max_primitives);

  std::set<uint32_t> deadnodes;
  collapse_nodes(bvh, 0, deadnodes);
  remove_deadnodes(bvh, deadnodes);

  return bvh;
}

bvh_t build_bvh_ploc(const std::vector<math::aabb_t> &aabbs, uint32_t grid_dim,
                     uint32_t log_bits, uint32_t search_radius) {
  bvh_t bvh{};

  std::vector<math::vec3> centers;
  for (const auto &aabb : aabbs) centers.push_back(aabb.center());

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
        math::vec3{float(grid_dim - 1)},
        math::max(math::vec3{0},
                  (centers[i] - center_aabb.min) *
                      (math::vec3{float(grid_dim)} /
                       math::vec3{center_aabb.max - center_aabb.min})));
    morton_codes[i] = encode_morton(grid_position.x, grid_position.y,
                                    grid_position.z, log_bits);
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

  bvh.nodes                = std::vector<node_t>(2 * aabbs.size() - 1);
  uint32_t insertion_index = bvh.nodes.size();

  while (current_nodes.size() > 1) {
    for (uint32_t i = 0; i < current_nodes.size(); i++)
      merge_index[i] = find_best_node(current_nodes, i, search_radius);
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

  fix_primitive_indices(bvh);

  std::set<uint32_t> deadnodes;
  collapse_nodes(bvh, 0, deadnodes);
  remove_deadnodes(bvh, deadnodes);

  return bvh;
}

bvh_t build_bvh_sweep_sah(const std::vector<math::aabb_t> &aabbs,
                          uint32_t min_primitives, uint32_t max_primitives) {
  bvh_t bvh{};

  std::vector<math::vec3> centers;
  for (const auto &aabb : aabbs) centers.push_back(aabb.center());

  uint32_t primitive_count = aabbs.size();

  std::vector<uint32_t> sorted_prim_indices[3];
  for (uint32_t i = 0; i < primitive_count; i++) {
    sorted_prim_indices[0].push_back(i);
    sorted_prim_indices[1].push_back(i);
    sorted_prim_indices[2].push_back(i);
  }

  std::sort(sorted_prim_indices[0].begin(), sorted_prim_indices[0].end(),
            [&](uint32_t a, uint32_t b) -> bool {
              return centers[a][0] < centers[b][0];
            });
  std::sort(sorted_prim_indices[1].begin(), sorted_prim_indices[1].end(),
            [&](uint32_t a, uint32_t b) -> bool {
              return centers[a][1] < centers[b][1];
            });
  std::sort(sorted_prim_indices[2].begin(), sorted_prim_indices[2].end(),
            [&](uint32_t a, uint32_t b) -> bool {
              return centers[a][2] < centers[b][2];
            });

  node_t root;
  root.index      = 0;
  root.prim_count = primitive_count;
  bvh.nodes.push_back(root);

  std::vector<bool>  move_left(primitive_count);
  std::vector<float> right_costs(primitive_count);

  update_node_bounds(bvh, sorted_prim_indices[0], 0, aabbs.data());
  try_split_node_sweep_sah(bvh, 0, aabbs.data(), centers.data(),
                           sorted_prim_indices, move_left, right_costs,
                           min_primitives, max_primitives);

  bvh.prim_indices = sorted_prim_indices[0];

  uint32_t depth = depth_of_bvh(bvh);
  if (depth >= 64) {
    uint32_t min = std::numeric_limits<uint32_t>::max();
    uint32_t max = 0;
    for (const auto &node : bvh.nodes) {
      if (!node.is_leaf()) continue;
      min = std::min(node.prim_count, min);
      max = std::max(node.prim_count, max);
    }
    std::cout << "min primitives: " << min << '\n';
    std::cout << "max primitives: " << max << '\n';
    std::cout << depth << ' ' << bvh.nodes.size() << '\n';
    throw std::runtime_error("depth >= 64");
  }

  std::set<uint32_t> deadnodes;
  collapse_nodes(bvh, 0, deadnodes);
  remove_deadnodes(bvh, deadnodes);

  return bvh;
}

std::vector<uint32_t> get_parents(const bvh_t &bvh) {
  std::vector<uint32_t> parents(bvh.nodes.size());
  for (uint32_t i = 0; i < bvh.nodes.size(); i++) {
    const node_t &node = bvh.nodes[i];
    if (node.is_leaf()) continue;
    parents[node.index + 0] = i;
    parents[node.index + 1] = i;
  }
  return parents;
}

bool is_left_sibling(uint32_t index) { return index % 2 == 1; }

uint32_t get_left_sibling_index(uint32_t index) {
  return is_left_sibling(index) ? index : index - 1;
}

uint32_t sibling(uint32_t index) {
  return is_left_sibling(index) ? index + 1 : index - 1;
}

struct candidate_t {
  uint32_t index = 0;
  float    cost  = -math::infinity;
  bool operator>(const candidate_t &other) const { return cost > other.cost; }
};

struct reinsertion_t {
  uint32_t from      = 0;
  uint32_t to        = 0;
  float    area_diff = 0;
  bool     operator>(const reinsertion_t &other) const {
    return area_diff > other.area_diff;
  }
};

std::vector<candidate_t> find_candidates(bvh_t &bvh, uint32_t target_count) {
  const uint32_t node_count =
      std::min((uint32_t)bvh.nodes.size(), target_count + 1);
  std::vector<candidate_t> candidates;
  for (uint32_t i = 1; i < node_count; i++)
    candidates.push_back(candidate_t{i, bvh.nodes[i].aabb().area()});
  std::make_heap(candidates.begin(), candidates.end(), std::greater<>{});
  for (uint32_t i = node_count; i < bvh.nodes.size(); i++) {
    float cost = bvh.nodes[i].aabb().area();
    if (candidates.front().cost < cost) {
      std::pop_heap(candidates.begin(), candidates.end(), std::greater<>{});
      candidates.back() = candidate_t{i, cost};
      std::push_heap(candidates.begin(), candidates.end(), std::greater<>{});
    }
  }
  return candidates;
}

reinsertion_t find_reinsertion(const bvh_t &bvh, uint32_t index,
                               const std::vector<uint32_t> &parents) {
  assert(index != 0);
  reinsertion_t best_reinsertion{.from = index};
  float         node_area     = bvh.nodes[index].aabb().area();
  float         parent_area   = bvh.nodes[parents[index]].aabb().area();
  float         area_diff     = parent_area;
  uint32_t      sibling_index = sibling(index);
  math::aabb_t  pivot_aabb    = bvh.nodes[sibling_index].aabb();
  uint32_t      parent_index  = parents[index];
  uint32_t      pivot_index   = parent_index;

  std::vector<std::pair<float, uint32_t>> stack;
  do {
    stack.emplace_back(area_diff, sibling_index);
    while (!stack.empty()) {
      auto top = stack.back();
      stack.pop_back();
      if (top.first - node_area <= best_reinsertion.area_diff) continue;

      const node_t &dst_node = bvh.nodes[top.second];
      float merged_area = dst_node.aabb().grow(bvh.nodes[index].aabb()).area();
      float reinsert_area = top.first - merged_area;
      if (reinsert_area > best_reinsertion.area_diff) {
        best_reinsertion.to        = top.second;
        best_reinsertion.area_diff = reinsert_area;
      }

      if (!dst_node.is_leaf()) {
        float child_area = reinsert_area + dst_node.aabb().area();
        stack.emplace_back(child_area, dst_node.index + 0);
        stack.emplace_back(child_area, dst_node.index + 1);
      }
    }

    if (pivot_index != parent_index) {
      pivot_aabb.grow(bvh.nodes[sibling_index].aabb());
      area_diff += bvh.nodes[pivot_index].aabb().area() - pivot_aabb.area();
    }

    sibling_index = sibling(pivot_index);
    pivot_index   = parents[pivot_index];
  } while (pivot_index != 0);

  if (best_reinsertion.to == sibling(best_reinsertion.from) ||
      best_reinsertion.to == parents[best_reinsertion.from])
    best_reinsertion = {};
  return best_reinsertion;
}

void refit_from(bvh_t &bvh, uint32_t index,
                const std::vector<uint32_t> &parents) {
  do {
    node_t &node = bvh.nodes[index];
    if (!node.is_leaf()) {
      math::aabb_t aabb = bvh.nodes[node.index + 0].aabb().grow(
          bvh.nodes[node.index + 1].aabb());
      node.min = aabb.min;
      node.max = aabb.max;
    }
    index = parents[index];
  } while (index != 0);
}

void reinsert_node(bvh_t &bvh, uint32_t from, uint32_t to,
                   std::vector<uint32_t> &parents) {
  uint32_t sibling_index = sibling(from);
  uint32_t parent_index  = parents[from];
  node_t   sibling_node  = bvh.nodes[sibling_index];
  node_t   dst_node      = bvh.nodes[to];

  bvh.nodes[to].index      = get_left_sibling_index(from);
  bvh.nodes[to].prim_count = 0;
  bvh.nodes[sibling_index] = dst_node;
  bvh.nodes[parent_index]  = sibling_node;

  if (!dst_node.is_leaf()) {
    parents[dst_node.index + 0] = sibling_index;
    parents[dst_node.index + 1] = sibling_index;
  }
  if (!sibling_node.is_leaf()) {
    parents[sibling_node.index + 0] = parent_index;
    parents[sibling_node.index + 1] = parent_index;
  }

  parents[sibling_index] = to;
  parents[from]          = to;
  refit_from(bvh, to, parents);
  refit_from(bvh, parent_index, parents);
}

std::array<uint32_t, 5> get_conflicts(uint32_t from, uint32_t to,
                                      const std::vector<uint32_t> &parents) {
  return {to, from, sibling(from), parents[to], parents[from]};
}

void layout_optimize(bvh_t &bvh) {
  std::vector<node_t>   nodes;
  std::vector<uint32_t> new_to_old(bvh.nodes.size());
  std::vector<uint32_t> old_to_new(bvh.nodes.size());
  uint32_t              stack[64], stack_top = 0;
  stack[stack_top++] = 0;
  node_t root        = bvh.nodes[0];
  nodes.push_back(root);
  while (stack_top) {
    node_t node = bvh.nodes[stack[--stack_top]];
    if (!node.is_leaf()) {
      node_t   left           = bvh.nodes[node.index + 0];
      uint32_t new_left_index = nodes.size();
      nodes.push_back(left);
      node_t   right           = bvh.nodes[node.index + 1];
      uint32_t new_right_index = nodes.size();
      nodes.push_back(right);
      old_to_new[node.index + 0] = new_left_index;
      old_to_new[node.index + 1] = new_right_index;
      stack[stack_top++]         = node.index + 1;
      stack[stack_top++]         = node.index + 0;
    }
  }
  for (uint32_t i = 0; i < bvh.nodes.size(); i++) {
    node_t &node = nodes[i];
    if (node.is_leaf()) continue;
    uint32_t new_index = i;
    uint32_t old_index = new_to_old[new_index];
    node.index         = old_to_new[node.index];
  }
  bvh.nodes = nodes;
}

void reinsertion_optimize(bvh_t &bvh, float batch_size_ratio,
                          uint32_t max_itr) {
  std::vector<uint32_t> parents = get_parents(bvh);
  uint32_t              batch_size =
      std::max(uint32_t{1}, uint32_t(bvh.nodes.size() * batch_size_ratio));

  std::vector<reinsertion_t> reinsertions;
  std::vector<bool>          touched(bvh.nodes.size());

  for (uint32_t itr = 0; itr < max_itr; itr++) {
    std::vector<candidate_t> candidates = find_candidates(bvh, batch_size);

    std::fill(touched.begin(), touched.end(), false);
    reinsertions.resize(candidates.size());
    for (uint32_t i = 0; i < candidates.size(); i++) {
      reinsertions[i] = find_reinsertion(bvh, candidates[i].index, parents);
    }

    reinsertions.erase(
        std::remove_if(reinsertions.begin(), reinsertions.end(),
                       [](reinsertion_t &r) { return r.area_diff <= 0; }),
        reinsertions.end());
    std::sort(reinsertions.begin(), reinsertions.end(), std::greater<>{});

    for (auto &reinsertion : reinsertions) {
      auto conflicts = get_conflicts(reinsertion.from, reinsertion.to, parents);
      if (std::any_of(conflicts.begin(), conflicts.end(),
                      [&](uint32_t i) { return touched[i]; }))
        continue;
      for (auto conflict : conflicts) touched[conflict] = true;
      reinsert_node(bvh, reinsertion.from, reinsertion.to, parents);
    }
  }
  layout_optimize(bvh);
}

std::pair<bvh_t, std::vector<math::triangle_t>> build_bvh(
    const model::raw_mesh_t &mesh) {
  bvh_t bvh{};

  std::vector<math::triangle_t> triangles =
      model::create_triangles_from_mesh(mesh);

  auto [aabbs, tri_indices] = presplit(triangles);

  bvh = build_bvh_binned_sah(aabbs);

  presplit_remove_indirection(bvh, tri_indices);
  presplit_remove_duplicates(bvh);

  return {bvh, triangles};
}

struct binary_reader_t {
  binary_reader_t(const std::filesystem::path &path)
      : _path(path), _file(path, std::ios::binary) {
    if (!_file.is_open()) {
      std::stringstream ss;
      ss << "Error: failed to open file " << _path.string();
      throw std::runtime_error(ss.str());
    }
  }

  ~binary_reader_t() { _file.close(); }

  size_t file_size() { return std::filesystem::file_size(_path); }

  // TODO: add error handling
  template <typename type_t>
  void read(type_t &val) {
    _file.read(reinterpret_cast<char *>(&val), sizeof(type_t));
  }

  std::filesystem::path _path;
  std::ifstream         _file;
};

struct binary_writer_t {
  binary_writer_t(const std::filesystem::path &path)
      : _path(path), _file(path, std::ios::binary) {
    if (!_file.is_open()) {
      std::stringstream ss;
      ss << "Error: failed to open file " << _path.string();
      throw std::runtime_error(ss.str());
    }
  }

  ~binary_writer_t() {
    flush();
    _file.close();
  }

  template <typename type_t>
  void write(const type_t &val) {
    const char *data = reinterpret_cast<const char *>(&val);
    for (size_t i = 0; i < sizeof(type_t); i++) {
      _buffer.push_back(data[i]);
    }
  }

  void flush() {
    _file.write(_buffer.data(), _buffer.size());
    _buffer.clear();
  }

  std::filesystem::path _path;
  std::ofstream         _file;
  std::vector<char>     _buffer;
};

void save(const bvh_t &bvh, const std::filesystem::path &path) {
  binary_writer_t writer{path};
  writer.write((uint32_t)bvh.nodes.size());
  for (auto &node : bvh.nodes) {
    writer.write(node);
  }
  writer.write((uint32_t)bvh.prim_indices.size());
  for (auto &index : bvh.prim_indices) {
    writer.write(index);
  }
}

bvh_t load(const std::filesystem::path &path) {
  bvh_t           bvh{};
  binary_reader_t reader{path};
  uint32_t        size;
  reader.read(size);
  bvh.nodes.resize(size);
  for (auto &node : bvh.nodes) {
    reader.read(node);
  }
  reader.read(size);
  bvh.prim_indices.resize(size);
  for (auto &index : bvh.prim_indices) {
    reader.read(index);
  }
  return bvh;
}

}  // namespace bvh
