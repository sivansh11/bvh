#include "bvh/bvh.hpp"
#include <iostream>

#define TINYBVH_USE_CUSTOM_VECTOR_TYPES
namespace tinybvh {
using bvhint2 = math::ivec2;
using bvhint3 = math::ivec3;
using bvhuint2 = math::uvec2;
using bvhvec2 = math::vec2;
using bvhvec3 = math::vec3;
using bvhvec4 = math::vec4;
using bvhdbl3 = math::dvec3;
using bvhmat4 = math::mat4;
} // namespace tinybvh
#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

namespace bvh {

bvh_t build_bvh(const model::raw_mesh_t &mesh) {
  bvh_t bvh{};
  for (uint32_t i = 0; i < mesh.indices.size(); i += 3) {
    bvh_triangle_t triangle{
        {mesh.vertices[mesh.indices[i + 0]].position, 0},
        {mesh.vertices[mesh.indices[i + 1]].position, 0},
        {mesh.vertices[mesh.indices[i + 2]].position, 0},
    };
    bvh.triangles.push_back(triangle);
  }

  tinybvh::BVH tiny_bvh{};
  tiny_bvh.Build(reinterpret_cast<tinybvh::bvhvec4 *>(bvh.triangles.data()),
                   static_cast<uint32_t>(bvh.triangles.size()));

  bvh.nodes.resize(tiny_bvh.usedNodes);
  static_assert(sizeof(tinybvh::BVH::BVHNode) == sizeof(node_t),
                "sizes should be same");
  std::memcpy(bvh.nodes.data(), tiny_bvh.bvhNode,
              tiny_bvh.usedNodes * sizeof(node_t));

  bvh.prim_indices.resize(bvh.triangles.size());
  std::memcpy(bvh.prim_indices.data(), tiny_bvh.primIdx,
              bvh.triangles.size() * sizeof(uint32_t));

  return bvh;
}

gpu_bvh_t build_gpu_bvh(const model::raw_mesh_t &mesh) {
  gpu_bvh_t gpu_bvh{};
  for (uint32_t i = 0; i < mesh.indices.size(); i += 3) {
    bvh_triangle_t triangle{
        {mesh.vertices[mesh.indices[i + 0]].position, 0},
        {mesh.vertices[mesh.indices[i + 1]].position, 0},
        {mesh.vertices[mesh.indices[i + 2]].position, 0},
    };
    gpu_bvh.triangles.push_back(triangle);
  }

  tinybvh::BVH_GPU bvh{};
  bvh.BuildHQ(reinterpret_cast<tinybvh::bvhvec4 *>(gpu_bvh.triangles.data()),
              static_cast<uint32_t>(gpu_bvh.triangles.size()));

  gpu_bvh.nodes.resize(bvh.usedNodes);
  static_assert(sizeof(tinybvh::BVH_GPU::BVHNode) == sizeof(gpu_node_t),
                "sizes should be same");
  std::memcpy(gpu_bvh.nodes.data(), bvh.bvhNode,
              bvh.usedNodes * sizeof(gpu_node_t));

  gpu_bvh.prim_indices.resize(gpu_bvh.triangles.size());
  std::memcpy(gpu_bvh.prim_indices.data(), bvh.bvh.primIdx,
              gpu_bvh.triangles.size() * sizeof(uint32_t));

  return gpu_bvh;
}

uint32_t depth_of_node(const bvh_t &bvh, uint32_t node_id) {
  const node_t &node = bvh.nodes[node_id];
  if (node.is_leaf()) {
    return 1;
  } else {
    uint32_t children_max_depth = std::max(depth_of_node(bvh, node.index + 0),
                                           depth_of_node(bvh, node.index + 1));
    return children_max_depth + 1;
  }
  throw std::runtime_error("shouldnt reach here");
}

uint32_t depth_of_bvh(const bvh_t &bvh) { return depth_of_node(bvh, 0); }

float cost_of_node(const bvh_t &bvh, uint32_t node_index) {
  const node_t &node = bvh.nodes[node_index];
  if (node.is_leaf())
    return 1.1f * node.prim_count;
  const node_t &left = bvh.nodes[node.index + 0];
  const node_t &right = bvh.nodes[node.index + 1];
  float cost = left.aabb().area() * cost_of_node(bvh, node.index + 0) +
               right.aabb().area() * cost_of_node(bvh, node.index + 1);
  return 1.f + (cost / node.aabb().area());
}

float cost_of_bvh(const bvh_t &bvh) { return cost_of_node(bvh, 0); }

} // namespace bvh
