#ifndef BVH_BVH_HPP
#define BVH_BVH_HPP

#include <cstdint>
#include <vector>

#include "math/math.hpp"
#include "model/model.hpp"

namespace bvh {

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
static_assert(sizeof(gpu_node_t) == 64, "sizeof(node_t) should be 64 bytes");

struct triangle_t {
  math::vec4 v0;
  math::vec4 v1;
  math::vec4 v2;
};

struct gpu_bvh_t {
  std::vector<gpu_node_t> nodes;
  std::vector<triangle_t> triangles;
  std::vector<uint32_t> indices;
};

gpu_bvh_t build_gpu_bvh(const model::raw_mesh_t &mesh);

} // namespace bvh

#endif
