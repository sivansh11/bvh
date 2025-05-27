#include "bvh/traversal.hpp"
#include "bvh/bvh.hpp"
#include "math/math.hpp"
#include "math/triangle.hpp"

namespace bvh {

namespace utils {

float safe_inverse(float x) {
  static constexpr float epsilon = std::numeric_limits<float>::epsilon();
  if (std::abs(x) <= epsilon) {
    return x >= 0 ? 1.f / epsilon : -1.f / epsilon;
  }
  return 1.f / x;
}

} // namespace utils

ray_data_t ray_data_t::create(const bvh::ray_t &ray) {
  ray_data_t ray_data{};
  ray_data.origin = ray.origin;
  ray_data.direction = ray.direction;
  ray_data.inverse_direction = math::vec3{
      utils::safe_inverse(ray.direction.x),
      utils::safe_inverse(ray.direction.y),
      utils::safe_inverse(ray.direction.z),
  };
  ray_data.tmin = 0.0001;
  ray_data.tmax = math::infinity;
  return ray_data;
}

triangle_intersection_t triangle_intersect(const math::triangle_t &triangle,
                                           const ray_data_t &ray_data) {
  math::vec3 e1 = triangle.v0 - triangle.v1;
  math::vec3 e2 = triangle.v2 - triangle.v0;
  math::vec3 n = cross(e1, e2);

  math::vec3 c = triangle.v0 - ray_data.origin;
  math::vec3 r = cross(ray_data.direction, c);
  float inverse_det = 1.0f / dot(n, ray_data.direction);

  float u = dot(r, e2) * inverse_det;
  float v = dot(r, e1) * inverse_det;
  float w = 1.0f - u - v;

  triangle_intersection_t intersection;

  if (u >= 0 && v >= 0 && w >= 0) {
    float t = dot(n, c) * inverse_det;
    if (t >= ray_data.tmin && t <= ray_data.tmax) {
      intersection.is_intersect = true;
      intersection.t = t;
      intersection.u = u;
      intersection.v = v;
      intersection.w = w;
      return intersection;
    }
  }
  intersection.is_intersect = false;
  return intersection;
}

aabb_intersection_t aabb_intersect(const math::aabb_t &aabb,
                                   const ray_data_t &ray_data) {
  math::vec3 tmin = (aabb.min - ray_data.origin) * ray_data.inverse_direction;
  math::vec3 tmax = (aabb.max - ray_data.origin) * ray_data.inverse_direction;

  const math::vec3 old_tmin = tmin;
  const math::vec3 old_tmax = tmax;

  tmin = min(old_tmin, old_tmax);
  tmax = max(old_tmin, old_tmax);

  float _tmin =
      math::max(tmin[0], math::max(tmin[1], math::max(tmin[2], ray_data.tmin)));
  float _tmax =
      math::min(tmax[0], math::min(tmax[1], math::min(tmax[2], ray_data.tmax)));

  aabb_intersection_t aabb_intersection = {_tmin, _tmax};
  return aabb_intersection;
}

hit_t traverse(uint32_t *primitive_indices, node_t *nodes,
               math::triangle_t *triangles, ray_t &ray) {
  ray_data_t ray_data = ray_data_t::create(ray);

  hit_t hit = null_hit;

  uint32_t stack[32];
  stack[0] = 0;
  uint32_t stack_top = 1;

  while (true) {
    node_t node = nodes[stack[--stack_top]];
    if (!aabb_intersect(node.aabb, ray_data).did_intersect())
      continue;
    if (node.is_leaf) {
      for (uint32_t i = 0; i < node.primitive_count; i++) {
        uint32_t primitive_index =
            primitive_indices[node.as.leaf.first_primitive_index + i];
        math::triangle_t triangle = triangles[primitive_index];
        hit.primitive_intersection_count++;
        triangle_intersection_t intersection =
            triangle_intersect(triangle, ray_data);
        if (intersection.did_intersect()) {
          ray_data.tmax = intersection.t;
          hit.primitive_index = primitive_index;
          hit.t = intersection.t;
          hit.u = intersection.u;
          hit.v = intersection.v;
          hit.w = intersection.w;
        }
      }
    } else {
      stack[stack_top++] = node.as.internal.first_child_index;
      stack[stack_top++] = node.as.internal.first_child_index + 1;
    }
  }
  return hit;

  if (false) {

    bvh::node_t root = nodes[0];

    hit.node_intersection_count++;

    if (!aabb_intersect(root.aabb, ray_data).did_intersect())
      return hit;

    if (root.is_leaf) {
      for (uint32_t i = 0; i < root.primitive_count; i++) {
        uint32_t primitive_index =
            primitive_indices[root.as.leaf.first_primitive_index + i];
        math::triangle_t triangle = triangles[primitive_index];
        hit.primitive_intersection_count++;
        triangle_intersection_t intersection =
            triangle_intersect(triangle, ray_data);
        if (intersection.did_intersect()) {
          ray_data.tmax = intersection.t;
          hit.primitive_index = primitive_index;
          hit.t = intersection.t;
          hit.u = intersection.u;
          hit.v = intersection.v;
          hit.w = intersection.w;
        }
      }
      return hit;
    }

    uint32_t current = 1;

    uint32_t stack_top = 0;
    static const uint32_t STACK_SIZE = 16;
    uint32_t stack[STACK_SIZE];

    while (true) {
      const bvh::node_t left = nodes[current];
      const bvh::node_t right = nodes[current + 1];

      hit.node_intersection_count += 2;
      aabb_intersection_t left_intersect = aabb_intersect(left.aabb, ray_data);
      aabb_intersection_t right_intersect =
          aabb_intersect(right.aabb, ray_data);

      uint32_t start = 0;
      uint32_t end = 0;
      if (left_intersect.did_intersect() && bool(left.is_leaf)) {
        if (right_intersect.did_intersect() && bool(right.is_leaf)) {
          start = left.as.leaf.first_primitive_index;
          end = right.as.leaf.first_primitive_index + right.primitive_count;
        } else {
          start = left.as.leaf.first_primitive_index;
          end = left.as.leaf.first_primitive_index + left.primitive_count;
        }
      } else if (right_intersect.did_intersect() && bool(right.is_leaf)) {
        start = right.as.leaf.first_primitive_index;
        end = right.as.leaf.first_primitive_index + right.primitive_count;
      }
      for (uint32_t i = start; i < end; i++) {
        const uint32_t primitive_index = primitive_indices[i];
        math::triangle_t triangle = triangles[primitive_index];
        hit.primitive_intersection_count++;
        triangle_intersection_t intersection =
            triangle_intersect(triangle, ray_data);
        if (intersection.did_intersect()) {
          ray_data.tmax = intersection.t;
          hit.primitive_index = primitive_index;
          hit.t = intersection.t;
          hit.u = intersection.u;
          hit.v = intersection.v;
          hit.w = intersection.w;
        }
      }

      if (left_intersect.did_intersect() && !bool(left.is_leaf)) {
        if (right_intersect.did_intersect() && !bool(right.is_leaf)) {
          if (stack_top >= STACK_SIZE)
            return hit; // TODO: maybe raise an error ?
          if (left_intersect.tmin <= right_intersect.tmin) {
            current = left.as.internal.first_child_index;
            stack[stack_top++] = right.as.internal.first_child_index;
          } else {
            current = right.as.internal.first_child_index;
            stack[stack_top++] = left.as.internal.first_child_index;
          }
        } else {
          current = left.as.internal.first_child_index;
        }
      } else {
        if (right_intersect.did_intersect() && !bool(right.is_leaf)) {
          current = right.as.internal.first_child_index;
        } else {
          if (stack_top == 0)
            return hit;
          current = stack[--stack_top];
        }
      }
    }

    return hit;
  }
}

} // namespace bvh
