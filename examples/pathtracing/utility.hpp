#ifndef UTILITY_HPP
#define UTILITY_HPP

#include "model/model.hpp"
#include "scene.hpp"

inline bool russian_roulette_terminate(sampler_t  &sampler,
                                       math::vec3 &throughput) {
  float p =
      std::min(1.0f, std::max({throughput.r, throughput.g, throughput.b}));
  if (sampler.randf() > p) return true;
  throughput /= p;
  return false;
}

inline float power_huristic(float a, float b) {
  if (math::isinf(a) || math::isnan(a)) return 0.5f;
  if (math::isinf(b) || math::isnan(b)) return 0.5f;
  if (a == 0.f && b == 0.f) return 0.5;
  float a2 = a * a;
  float b2 = b * b;
  return a2 / (a2 + b2);
}

inline math::vec3 get_world_normal(const scene_t          &scene,
                                   const tlas::instance_t &instance,
                                   const tlas::hit_t       hit) {
  math::vec3 local_normal = scene.blases[instance.blas_index]
                                .triangles[hit.blas_hit.prim_index]
                                .normal();
  math::vec3 normal = math::normalize(
      math::transpose(math::mat3{instance.inv_transform}) * local_normal);
  return normal;
}

inline model::vertex_t interpolate_vertex(const scene_t          &scene,
                                          const tlas::instance_t &instance,
                                          const tlas::hit_t      &hit) {
  uint32_t    mesh_index = scene.instance_to_mesh_index[hit.instance_index];
  const auto &raw_mesh   = scene.raw_meshes[mesh_index];
  uint32_t    tri_idx    = hit.blas_hit.prim_index;
  uint32_t    i0         = raw_mesh.indices[tri_idx * 3 + 0];
  uint32_t    i1         = raw_mesh.indices[tri_idx * 3 + 1];
  uint32_t    i2         = raw_mesh.indices[tri_idx * 3 + 2];
  const auto &v0         = raw_mesh.vertices[i0];
  const auto &v1         = raw_mesh.vertices[i1];
  const auto &v2         = raw_mesh.vertices[i2];
  float       u          = hit.blas_hit.u;
  float       v          = hit.blas_hit.v;
  float       w          = 1.0f - u - v;

  model::vertex_t interpolated;
  interpolated.position = w * v0.position + u * v1.position + v * v2.position;
  interpolated.normal   = w * v0.normal + u * v1.normal + v * v2.normal;
  interpolated.uv       = w * v0.uv + u * v1.uv + v * v2.uv;
  interpolated.tangent  = w * v0.tangent + u * v1.tangent + v * v2.tangent;
  interpolated.bi_tangent =
      w * v0.bi_tangent + u * v1.bi_tangent + v * v2.bi_tangent;

  interpolated.position =
      instance.transform * math::vec4{interpolated.position, 1.f};
  math::mat3 normal_mat = math::transpose(math::mat3{instance.inv_transform});
  interpolated.normal   = math::normalize(normal_mat * interpolated.normal);
  interpolated.tangent  = math::normalize(normal_mat * interpolated.tangent);
  interpolated.bi_tangent =
      math::normalize(normal_mat * interpolated.bi_tangent);

  return interpolated;
}

#endif
