#ifndef BVH_BVH_HPP
#define BVH_BVH_HPP

#include <cstdint>
#include <vector>

#include "math/math.hpp"
#include "model/model.hpp"

namespace bvh {

struct node_t {
  math::vec3 aabbMin;
  uint32_t leftFirst; // 16 bytes
  math::vec3 aabbMax;
  uint32_t triCount; // 16 bytes, total: 32 bytes
};
static_assert(sizeof(node_t) == 32, "sizeof(node_t) should be 32 bytes");

struct gpu_node_t {
  math::vec3 lmin;
  uint32_t left;
  math::vec3 lmax;
  uint32_t right;
  math::vec3 rmin;
  uint32_t triCount;
  math::vec3 rmax;
  uint32_t firstTri;
};
static_assert(sizeof(gpu_node_t) == 64,
              "sizeof(gpu_node_t) should be 64 bytes");

struct triangle_t {
  math::vec4 v0;
  math::vec4 v1;
  math::vec4 v2;
};

struct bvh_t {
  std::vector<node_t> nodes;
  std::vector<triangle_t> triangles;
  std::vector<uint32_t> indices;
};

struct gpu_bvh_t {
  std::vector<gpu_node_t> nodes;
  std::vector<triangle_t> triangles;
  std::vector<uint32_t> indices;
};

bvh_t build_bvh(const model::raw_mesh_t &mesh);
gpu_bvh_t build_gpu_bvh(const model::raw_mesh_t &mesh);

} // namespace bvh

#endif
