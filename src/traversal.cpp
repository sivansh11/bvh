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

hit_t intersect_bvh(const bvh::node_t *nodes, const uint32_t *indices,
                    const math::triangle_t *triangles, ray_t ray) {
  static const uint32_t stack_size = 128;
  uint32_t              stack[stack_size];

  hit_t hit = hit_t();

  uint32_t stack_top = 0;

  bvh::node_t root = nodes[0];
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

    hit.node_intersections += 1;
    aabb_hit_t left_hit  = intersect_aabb(left.min, left.max, ray);
    aabb_hit_t right_hit = intersect_aabb(right.min, right.max, ray);

    uint32_t start = 0;
    uint32_t end   = 0;
    if (left_hit.did_intersect() && left.is_leaf()) {
      if (right_hit.did_intersect() && right.is_leaf()) {
        assert(left.index + left.prim_count == right.index);
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

inline aabb_hit_t intersect_soa_aabb(const soa_bvh_t &soa_bvh,
                                     const uint32_t node, const ray_t ray) {
  const math::vec4 min     = soa_bvh.mins[node];
  const math::vec4 max     = soa_bvh.maxs[node];
  const math::vec4 origin  = math::vec4(ray.origin, 0.f);
  const math::vec4 inv_dir = math::vec4(ray.inverse_direction, 1.f);

  const math::vec4 tmin =
      math::min((min - origin) * inv_dir, (max - origin) * inv_dir);
  const math::vec4 tmax =
      math::max((min - origin) * inv_dir, (max - origin) * inv_dir);
  float _tmin =
      math::max(tmin[0], math::max(tmin[1], math::max(tmin[2], ray.tmin)));
  float _tmax =
      math::min(tmax[0], math::min(tmax[1], math::min(tmax[2], ray.tmax)));
  aabb_hit_t hit = aabb_hit_t(_tmin, _tmax);
  return hit;
}

hit_t intersect_soa_bvh(const soa_bvh_t        &soa_bvh,
                        const math::triangle_t *triangles, ray_t ray) {
  static const uint32_t stack_size = 128;
  uint32_t              stack[stack_size];

  hit_t hit = hit_t();

  uint32_t stack_top = 0;

  if (!intersect_soa_aabb(soa_bvh, 0, ray).did_intersect()) return hit;

  if (soa_bvh.is_leaf(0)) {
    for (uint32_t i = 0; i < soa_bvh.prim_counts[0]; i++) {
      const uint32_t triangle_index =
          soa_bvh.prim_indices[soa_bvh.indexes[0] + i];
      const math::triangle_t triangle     = triangles[triangle_index];
      triangle_hit_t         triangle_hit = intersect_triangle(triangle, ray);
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

  uint32_t current = soa_bvh.indexes[0];

  while (true) {
    const uint32_t left  = current + 0;
    const uint32_t right = current + 1;

    hit.node_intersections += 1;
    aabb_hit_t left_hit  = intersect_soa_aabb(soa_bvh, left, ray);
    aabb_hit_t right_hit = intersect_soa_aabb(soa_bvh, right, ray);

    uint32_t start = 0;
    uint32_t end   = 0;
    if (left_hit.did_intersect() && soa_bvh.is_leaf(left)) {
      if (right_hit.did_intersect() && soa_bvh.is_leaf(right)) {
        assert(soa_bvh.indexes[left] + soa_bvh.prim_counts[left] ==
               soa_bvh.indexes[right]);
        start = soa_bvh.indexes[left];
        end   = soa_bvh.indexes[right] + soa_bvh.prim_counts[right];
      } else {
        start = soa_bvh.indexes[left];
        end   = soa_bvh.indexes[left] + soa_bvh.prim_counts[left];
      }
    } else {
      if (right_hit.did_intersect() && soa_bvh.is_leaf(right)) {
        start = soa_bvh.indexes[right];
        end   = soa_bvh.indexes[right] + soa_bvh.prim_counts[right];
      }
    }
    for (uint32_t index = start; index < end; index++) {
      const uint32_t         triangle_index = soa_bvh.prim_indices[index];
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

    if (left_hit.did_intersect() && !soa_bvh.is_leaf(left)) {
      if (right_hit.did_intersect() && !soa_bvh.is_leaf(right)) {
        if (stack_top >= stack_size) return hit;
        if (left_hit.tmin <= right_hit.tmin) {
          current            = soa_bvh.indexes[left];
          stack[stack_top++] = soa_bvh.indexes[right];
        } else {
          current            = soa_bvh.indexes[right];
          stack[stack_top++] = soa_bvh.indexes[left];
        }
      } else {
        current = soa_bvh.indexes[left];
      }
    } else {
      if (right_hit.did_intersect() && !soa_bvh.is_leaf(right)) {
        current = soa_bvh.indexes[right];
      } else {
        if (stack_top == 0) return hit;
        current = stack[--stack_top];
      }
    }
  }
  return hit;
}

template <bool posX, bool posY, bool posZ>
static inline aabb_hit_t intersect_aabb_slab(const math::vec3 _min,  //
                                             const math::vec3 _max,  //
                                             const ray_t      ray,   //
                                             const float      rox,   //
                                             const float      roy,   //
                                             const float      roz) {
  const float nx    = posX ? _min.x : _max.x;
  const float ny    = posY ? _min.y : _max.y;
  const float nz    = posZ ? _min.z : _max.z;
  const float fx    = posX ? _max.x : _min.x;
  const float fy    = posY ? _max.y : _min.y;
  const float fz    = posZ ? _max.z : _min.z;
  const float t1x   = nx * ray.inverse_direction.x - rox;
  const float t1y   = ny * ray.inverse_direction.y - roy;
  const float t1z   = nz * ray.inverse_direction.z - roz;
  const float t2x   = fx * ray.inverse_direction.x - rox;
  const float t2y   = fy * ray.inverse_direction.y - roy;
  const float t2z   = fz * ray.inverse_direction.z - roz;
  const float _tmin = math::max(math::max(t1x, t1y), math::max(t1z, ray.tmin));
  const float _tmax = math::min(math::min(t2x, t2y), math::min(t2z, ray.tmax));
  return aabb_hit_t(_tmin, _tmax);
}

template <bool posX, bool posY, bool posZ>
hit_t intersect_bvh_slab(const bvh::node_t *nodes, const uint32_t *indices,
                         const math::triangle_t *triangles, ray_t ray) {
  static const uint32_t stack_size = 128;
  uint32_t              stack[stack_size];

  hit_t hit = hit_t();

  uint32_t stack_top = 0;

  const float rox = ray.origin.x * ray.inverse_direction.x;
  const float roy = ray.origin.y * ray.inverse_direction.y;
  const float roz = ray.origin.z * ray.inverse_direction.z;

  bvh::node_t root = nodes[0];
  if (!intersect_aabb_slab<posX,   //
                           posY,   //
                           posZ>(  //
           root.min,               //
           root.max,               //
           ray,                    //
           rox,                    //
           roy,                    //
           roz)
           .did_intersect())
    return hit;

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

    hit.node_intersections += 1;
    aabb_hit_t left_hit = intersect_aabb_slab<posX, posY, posZ>(
        left.min, left.max, ray, rox, roy, roz);
    aabb_hit_t right_hit = intersect_aabb_slab<posX, posY, posZ>(
        right.min, right.max, ray, rox, roy, roz);

    uint32_t start = 0;
    uint32_t end   = 0;
    if (left_hit.did_intersect() && left.is_leaf()) {
      if (right_hit.did_intersect() && right.is_leaf()) {
        assert(left.index + left.prim_count == right.index);
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

template hit_t intersect_bvh_slab<false, false, false>(const bvh::node_t *,
                                                       const uint32_t *,
                                                       const math::triangle_t *,
                                                       ray_t);
template hit_t intersect_bvh_slab<true, false, false>(const bvh::node_t *,
                                                      const uint32_t *,
                                                      const math::triangle_t *,
                                                      ray_t);
template hit_t intersect_bvh_slab<false, true, false>(const bvh::node_t *,
                                                      const uint32_t *,
                                                      const math::triangle_t *,
                                                      ray_t);
template hit_t intersect_bvh_slab<false, false, true>(const bvh::node_t *,
                                                      const uint32_t *,
                                                      const math::triangle_t *,
                                                      ray_t);
template hit_t intersect_bvh_slab<true, true, false>(const bvh::node_t *,
                                                     const uint32_t *,
                                                     const math::triangle_t *,
                                                     ray_t);
template hit_t intersect_bvh_slab<true, false, true>(const bvh::node_t *,
                                                     const uint32_t *,
                                                     const math::triangle_t *,
                                                     ray_t);
template hit_t intersect_bvh_slab<false, true, true>(const bvh::node_t *,
                                                     const uint32_t *,
                                                     const math::triangle_t *,
                                                     ray_t);
template hit_t intersect_bvh_slab<true, true, true>(const bvh::node_t *,
                                                    const uint32_t *,
                                                    const math::triangle_t *,
                                                    ray_t);

hit_t intersect_bvh_slab(const bvh::node_t *nodes, const uint32_t *indices,
                         const math::triangle_t *triangles, ray_t ray) {
  const uint32_t sign_bits = (ray.direction.x >= 0.f ? 1u : 0u) |
                             (ray.direction.y >= 0.f ? 2u : 0u) |
                             (ray.direction.z >= 0.f ? 4u : 0u);
  switch (sign_bits) {
    case 0:
      return intersect_bvh_slab<false, false, false>(nodes, indices, triangles,
                                                     ray);
    case 1:
      return intersect_bvh_slab<true, false, false>(nodes, indices, triangles,
                                                    ray);
    case 2:
      return intersect_bvh_slab<false, true, false>(nodes, indices, triangles,
                                                    ray);
    case 3:
      return intersect_bvh_slab<true, true, false>(nodes, indices, triangles,
                                                   ray);
    case 4:
      return intersect_bvh_slab<false, false, true>(nodes, indices, triangles,
                                                    ray);
    case 5:
      return intersect_bvh_slab<true, false, true>(nodes, indices, triangles,
                                                   ray);
    case 6:
      return intersect_bvh_slab<false, true, true>(nodes, indices, triangles,
                                                   ray);
    default:
      return intersect_bvh_slab<true, true, true>(nodes, indices, triangles,
                                                  ray);
  }
}

template <bool posX, bool posY, bool posZ>
hit_t intersect_soa_bvh_slab(const soa_bvh_t        &soa_bvh,
                             const math::triangle_t *triangles, ray_t ray) {
  static const uint32_t stack_size = 128;
  uint32_t              stack[stack_size];

  hit_t hit = hit_t();

  uint32_t stack_top = 0;

  const float rox = ray.origin.x * ray.inverse_direction.x;
  const float roy = ray.origin.y * ray.inverse_direction.y;
  const float roz = ray.origin.z * ray.inverse_direction.z;

  if (!intersect_aabb_slab<posX, posY, posZ>(math::vec3(soa_bvh.mins[0]),
                                             math::vec3(soa_bvh.maxs[0]), ray,
                                             rox, roy, roz)
           .did_intersect())
    return hit;

  if (soa_bvh.is_leaf(0)) {
    for (uint32_t i = 0; i < soa_bvh.prim_counts[0]; i++) {
      const uint32_t triangle_index =
          soa_bvh.prim_indices[soa_bvh.indexes[0] + i];
      const math::triangle_t triangle     = triangles[triangle_index];
      triangle_hit_t         triangle_hit = intersect_triangle(triangle, ray);
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

  uint32_t current = soa_bvh.indexes[0];

  while (true) {
    const uint32_t left  = current + 0;
    const uint32_t right = current + 1;

    hit.node_intersections += 1;
    aabb_hit_t left_hit = intersect_aabb_slab<posX, posY, posZ>(
        math::vec3(soa_bvh.mins[left]), math::vec3(soa_bvh.maxs[left]), ray,
        rox, roy, roz);
    aabb_hit_t right_hit = intersect_aabb_slab<posX, posY, posZ>(
        math::vec3(soa_bvh.mins[right]), math::vec3(soa_bvh.maxs[right]), ray,
        rox, roy, roz);

    uint32_t start = 0;
    uint32_t end   = 0;
    if (left_hit.did_intersect() && soa_bvh.is_leaf(left)) {
      if (right_hit.did_intersect() && soa_bvh.is_leaf(right)) {
        assert(soa_bvh.indexes[left] + soa_bvh.prim_counts[left] ==
               soa_bvh.indexes[right]);
        start = soa_bvh.indexes[left];
        end   = soa_bvh.indexes[right] + soa_bvh.prim_counts[right];
      } else {
        start = soa_bvh.indexes[left];
        end   = soa_bvh.indexes[left] + soa_bvh.prim_counts[left];
      }
    } else {
      if (right_hit.did_intersect() && soa_bvh.is_leaf(right)) {
        start = soa_bvh.indexes[right];
        end   = soa_bvh.indexes[right] + soa_bvh.prim_counts[right];
      }
    }
    for (uint32_t index = start; index < end; index++) {
      const uint32_t         triangle_index = soa_bvh.prim_indices[index];
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

    if (left_hit.did_intersect() && !soa_bvh.is_leaf(left)) {
      if (right_hit.did_intersect() && !soa_bvh.is_leaf(right)) {
        if (stack_top >= stack_size) return hit;
        if (left_hit.tmin <= right_hit.tmin) {
          current            = soa_bvh.indexes[left];
          stack[stack_top++] = soa_bvh.indexes[right];
        } else {
          current            = soa_bvh.indexes[right];
          stack[stack_top++] = soa_bvh.indexes[left];
        }
      } else {
        current = soa_bvh.indexes[left];
      }
    } else {
      if (right_hit.did_intersect() && !soa_bvh.is_leaf(right)) {
        current = soa_bvh.indexes[right];
      } else {
        if (stack_top == 0) return hit;
        current = stack[--stack_top];
      }
    }
  }
  return hit;
}

template hit_t intersect_soa_bvh_slab<false, false, false>(
    const soa_bvh_t &, const math::triangle_t *, ray_t);
template hit_t intersect_soa_bvh_slab<true, false, false>(
    const soa_bvh_t &, const math::triangle_t *, ray_t);
template hit_t intersect_soa_bvh_slab<false, true, false>(
    const soa_bvh_t &, const math::triangle_t *, ray_t);
template hit_t intersect_soa_bvh_slab<false, false, true>(
    const soa_bvh_t &, const math::triangle_t *, ray_t);
template hit_t intersect_soa_bvh_slab<true, true, false>(
    const soa_bvh_t &, const math::triangle_t *, ray_t);
template hit_t intersect_soa_bvh_slab<true, false, true>(
    const soa_bvh_t &, const math::triangle_t *, ray_t);
template hit_t intersect_soa_bvh_slab<false, true, true>(
    const soa_bvh_t &, const math::triangle_t *, ray_t);
template hit_t intersect_soa_bvh_slab<true, true, true>(
    const soa_bvh_t &, const math::triangle_t *, ray_t);

hit_t intersect_soa_bvh_slab(const soa_bvh_t        &soa_bvh,
                             const math::triangle_t *triangles, ray_t ray) {
  const uint32_t sign_bits = (ray.direction.x >= 0.f ? 1u : 0u) |
                             (ray.direction.y >= 0.f ? 2u : 0u) |
                             (ray.direction.z >= 0.f ? 4u : 0u);
  switch (sign_bits) {
    case 0:
      return intersect_soa_bvh_slab<false, false, false>(soa_bvh, triangles,
                                                         ray);
    case 1:
      return intersect_soa_bvh_slab<true, false, false>(soa_bvh, triangles,
                                                        ray);
    case 2:
      return intersect_soa_bvh_slab<false, true, false>(soa_bvh, triangles,
                                                        ray);
    case 3:
      return intersect_soa_bvh_slab<true, true, false>(soa_bvh, triangles, ray);
    case 4:
      return intersect_soa_bvh_slab<false, false, true>(soa_bvh, triangles,
                                                        ray);
    case 5:
      return intersect_soa_bvh_slab<true, false, true>(soa_bvh, triangles, ray);
    case 6:
      return intersect_soa_bvh_slab<false, true, true>(soa_bvh, triangles, ray);
    default:
      return intersect_soa_bvh_slab<true, true, true>(soa_bvh, triangles, ray);
  }
}

}  // namespace bvh
