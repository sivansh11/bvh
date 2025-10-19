#include "bvh/traversal.hpp"

namespace bvh {

triangle_hit_t intersect_triangle(const math::triangle_t triangle,
                                  const ray_t            ray) {
  const math::vec3 e1 = math::vec3{triangle.v0} - math::vec3{triangle.v1};
  const math::vec3 e2 = math::vec3{triangle.v2} - math::vec3{triangle.v0};
  const math::vec3 n  = cross(e1, e2);

  const math::vec3 c           = math::vec3{triangle.v0} - ray.origin;
  const math::vec3 r           = cross(ray.direction, c);
  const float      inverse_det = 1.f / dot(n, ray.direction);  // could nan ?

  float u = dot(r, e2) * inverse_det;
  float v = dot(r, e1) * inverse_det;
  float w = 1.f - u - v;

  if (u >= 0 && v >= 0 && w >= 0) {
    float t = dot(n, c) * inverse_det;
    if (t > ray.tmin && t < ray.tmax) {
      triangle_hit_t hit;
      hit.t              = t;
      hit.u              = u;
      hit.v              = v;
      hit._did_intersect = true;
      return hit;
    }
  }
  triangle_hit_t hit;
  hit._did_intersect = false;
  return hit;
}

aabb_hit_t intersect_aabb(const math::vec3 _min, const math::vec3 _max,
                          const ray_t ray) {
  math::vec3       tmin     = (_min - ray.origin) * ray.inverse_direction;
  math::vec3       tmax     = (_max - ray.origin) * ray.inverse_direction;
  const math::vec3 old_tmin = tmin;
  const math::vec3 old_tmax = tmax;
  tmin                      = min(old_tmin, old_tmax);
  tmax                      = max(old_tmin, old_tmax);
  float _tmin =
      math::max(tmin[0], math::max(tmin[1], math::max(tmin[2], ray.tmin)));
  float _tmax =
      math::min(tmax[0], math::min(tmax[1], math::min(tmax[2], ray.tmax)));
  aabb_hit_t hit = aabb_hit_t(_tmin, _tmax);
  return hit;
}

hit_t intersect_bvh(bvh::node_t *nodes, uint32_t *indices,
                    math::triangle_t *triangles, ray_t ray) {
  static const uint32_t stack_size = 16;
  uint32_t              stack[stack_size];

  hit_t hit = hit_t();

  uint32_t stack_top = 0;

  bvh::node_t root = nodes[0];
  hit.node_intersections++;
  if (!intersect_aabb(root.min, root.max, ray).did_intersect()) return hit;

  if (root.is_leaf()) {
    for (uint32_t i = 0; i < root.prim_count; i++) {
      const uint32_t         triangle_index = indices[root.index + i];
      const math::triangle_t triangle       = triangles[triangle_index];
      triangle_hit_t         triangle_hit   = intersect_triangle(triangle, ray);
      hit.triangle_intersections++;
      if (triangle_hit.did_intersect()) {
        ray.tmax       = triangle_hit.t;
        hit.prim_index = triangle_index;
        hit.t          = triangle_hit.t;
        hit.u          = triangle_hit.u;
        hit.v          = triangle_hit.v;
      }
    }
    return hit;
  }

  uint32_t current = root.index;

  while (true) {
    bvh::node_t left  = nodes[current + 0];
    bvh::node_t right = nodes[current + 1];

    hit.node_intersections += 2;
    aabb_hit_t left_hit  = intersect_aabb(left.min, left.max, ray);
    aabb_hit_t right_hit = intersect_aabb(right.min, right.max, ray);

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
      uint32_t               triangle_index = indices[index];
      const math::triangle_t triangle       = triangles[triangle_index];
      triangle_hit_t         triangle_hit   = intersect_triangle(triangle, ray);
      hit.triangle_intersections++;
      if (triangle_hit.did_intersect()) {
        ray.tmax       = triangle_hit.t;
        hit.prim_index = triangle_index;
        hit.t          = triangle_hit.t;
        hit.u          = triangle_hit.u;
        hit.v          = triangle_hit.v;
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

}  // namespace bvh
