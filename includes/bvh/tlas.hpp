#ifndef BVH_TLAS_HPP
#define BVH_TLAS_HPP

#include "bvh/bvh.hpp"
#include "bvh/traversal.hpp"
#include "math/aabb.hpp"
#include "math/math.hpp"
#include "math/triangle.hpp"

namespace tlas {

struct instance_t {
  math::mat4   transform;
  math::mat4   inv_transform;
  math::aabb_t aabb;
  uint32_t     blas_index;
};

struct blas_t {
  bvh::bvh_t                    bvh;
  std::vector<math::triangle_t> triangles;
};

struct hit_t {
  uint32_t   instance_index     = bvh::null_index;
  uint32_t   blas_intersections = 0;
  bvh::hit_t blas_hit;
  bool       did_intersect() {
    return instance_index != bvh::null_index && blas_hit.did_intersect();
  }
};

hit_t intersect_tlas(const bvh::node_t *nodes, const uint32_t *indices,
                     const instance_t *instances, const blas_t *blases,
                     bvh::ray_t ray);

}  // namespace tlas

#endif
