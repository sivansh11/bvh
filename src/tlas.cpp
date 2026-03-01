#include "bvh/tlas.hpp"

#include "bvh/traversal.hpp"
#include "glm/ext/quaternion_geometric.hpp"
#include "glm/geometric.hpp"

namespace tlas {

hit_t intersect_tlas(bvh::node_t *nodes, uint32_t *indices,
                     instance_t *instances, blas_t *blases, bvh::ray_t ray) {
  static const uint32_t stack_size = 16;
  uint32_t              stack[stack_size];

  hit_t hit = hit_t();

  uint32_t stack_top = 0;

  bvh::node_t root = nodes[0];
  hit.blas_hit.node_intersections++;
  if (!intersect_aabb(root.min, root.max, ray).did_intersect()) return hit;

  if (root.is_leaf()) {
    for (uint32_t i = 0; i < root.prim_count; i++) {
      const uint32_t   instance_index = indices[root.index + i];
      const instance_t instance       = instances[instance_index];
      blas_t          &blas           = blases[instance.blas_index];
      math::vec4       local_origin =
          instance.inv_transform * math::vec4{ray.origin, 1.f};
      math::vec4 local_direction = math::normalize(
          instance.inv_transform * math::vec4{ray.direction, 0.f});
      float      l         = math::length(local_direction);
      bvh::ray_t local_ray = bvh::ray_t::create(local_origin, local_direction);
      local_ray.tmax       = ray.tmax * l;
      local_ray.tmin       = ray.tmin * l;
      bvh::hit_t blas_hit  = bvh::intersect_bvh(
          blas.bvh.nodes.data(), blas.bvh.prim_indices.data(),
          blas.triangles.data(), local_ray);
      hit.blas_intersections++;
      if (blas_hit.did_intersect()) {
        ray.tmax           = blas_hit.t / l;
        hit.blas_hit       = blas_hit;
        hit.blas_hit.t     = ray.tmax;
        hit.instance_index = instance_index;
      }
    }
    return hit;
  }

  uint32_t current = root.index;

  while (true) {
    bvh::node_t left  = nodes[current + 0];
    bvh::node_t right = nodes[current + 1];

    hit.blas_hit.node_intersections += 2;
    bvh::aabb_hit_t left_hit  = bvh::intersect_aabb(left.min, left.max, ray);
    bvh::aabb_hit_t right_hit = bvh::intersect_aabb(right.min, right.max, ray);

    uint32_t start = 0;
    uint32_t end   = 0;
    if (left_hit.did_intersect() && left.is_leaf()) {
      if (right_hit.did_intersect() && right.is_leaf()) {
        start = left.index;
        end   = right.index + right.prim_count;
      } else {
        start = left.index;
        end   = left.index + left.prim_count;
      }
    } else {
      if (right_hit.did_intersect() && right.is_leaf()) {
        start = right.index;
        end   = right.index + right.prim_count;
      }
    }
    for (uint32_t index = start; index < end; index++) {
      const uint32_t   instance_index = indices[index];
      const instance_t instance       = instances[instance_index];
      blas_t          &blas           = blases[instance.blas_index];
      math::vec4       local_origin =
          instance.inv_transform * math::vec4{ray.origin, 1.f};
      math::vec4 local_direction = math::normalize(
          instance.inv_transform * math::vec4{ray.direction, 0.f});
      float      l         = math::length(local_direction);
      bvh::ray_t local_ray = bvh::ray_t::create(local_origin, local_direction);
      local_ray.tmax       = ray.tmax * l;
      local_ray.tmin       = ray.tmin * l;
      bvh::hit_t blas_hit  = bvh::intersect_bvh(
          blas.bvh.nodes.data(), blas.bvh.prim_indices.data(),
          blas.triangles.data(), local_ray);
      hit.blas_intersections++;
      if (blas_hit.did_intersect()) {
        ray.tmax           = blas_hit.t / l;
        hit.blas_hit       = blas_hit;
        hit.blas_hit.t     = ray.tmax;
        hit.instance_index = instance_index;
      }
    }

    if (left_hit.did_intersect() && !left.is_leaf()) {
      if (right_hit.did_intersect() && !right.is_leaf()) {
        if (stack_top >= stack_size) return hit;
        if (left_hit.tmin <= right_hit.tmin) {
          current            = left.index;
          stack[stack_top++] = right.index;
        } else {
          current            = right.index;
          stack[stack_top++] = left.index;
        }
      } else {
        current = left.index;
      }
    } else {
      if (right_hit.did_intersect() && !right.is_leaf()) {
        current = right.index;
      } else {
        if (stack_top == 0) return hit;
        current = stack[--stack_top];
      }
    }
  }
  return hit;
}

}  // namespace tlas
