#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include "bvh/traversal.hpp"
#include "sampler.hpp"
#include "scene.hpp"
#include "utility.hpp"

struct light_intersection_t {
  bool       is_valid = false;
  material_t material;
  math::vec3 wi;
  math::vec3 normal;
  float      dist_sq;
  float      cos_theta;
  float      cos_light;
  float      pdf_area;
};

inline light_intersection_t find_visible_light(const scene_t    &scene,     //
                                               const bvh::bvh_t &tlas,      //
                                               const math::vec3 &hit_pos,   //
                                               const math::vec3 &normal,    //
                                               const math::vec3 &wo,        //
                                               const material_t &material,  //
                                               sampler_t        &sampler,   //
                                               const math::vec2 &uv) {
  if (!scene.light_instance_indices.size()) return {};

  float sample = sampler.randf();
  auto  itr = std::lower_bound(scene.light_cdfs.begin(), scene.light_cdfs.end(),
                               sample);
  uint32_t light_record_index = std::distance(scene.light_cdfs.begin(), itr);
  light_record_index =
      std::min(light_record_index, (uint32_t)scene.light_cdfs.size() - 1);
  const light_record_t &light_record = scene.light_records[light_record_index];
  float                 discrete_probability = light_record.probability;
  uint32_t              light_instance_index = light_record.instance_index;
  const auto           &instance       = scene.instances[light_instance_index];
  const auto           &blas           = scene.blases[instance.blas_index];
  uint32_t              triangle_index = light_record.triangle_index;
  auto                  triangle       = blas.triangles[triangle_index];
  triangle.v0 = instance.transform * math::vec4{triangle.v0, 1.f};
  triangle.v1 = instance.transform * math::vec4{triangle.v1, 1.f};
  triangle.v2 = instance.transform * math::vec4{triangle.v2, 1.f};

  math::vec3 p_world      = sampler.triangle(triangle);
  math::vec3 light_normal = triangle.normal();

  math::vec3 L       = p_world - hit_pos;
  float      dist_sq = math::length2(L);
  float      dist    = std::sqrt(dist_sq);
  math::vec3 wi      = L / dist;

  float cos_theta = math::dot(normal, wi);
  float cos_light = math::dot(light_normal, -wi);

  if (cos_theta > 0.0f && cos_light > 0.0f) {
    math::vec3 offset_normal = math::dot(wi, normal) > 0.f ? normal : -normal;
    math::vec3 shadow_origin = hit_pos + offset_normal * epsilon;
    bvh::ray_t shadow_ray    = bvh::ray_t::create(shadow_origin, wi);
    shadow_ray.tmin          = epsilon;
    shadow_ray.tmax          = dist * 0.999f;
    auto shadow_hit          = tlas::intersect_tlas(
        tlas.nodes.data(), tlas.prim_indices.data(), scene.instances.data(),
        scene.blases.data(), shadow_ray);
    bool is_occluded = shadow_hit.did_intersect() &&
                       !(shadow_hit.instance_index == light_instance_index &&
                         shadow_hit.blas_hit.prim_index == triangle_index);
    if (!is_occluded) {
      light_intersection_t li{};
      li.is_valid  = true;
      li.material  = scene.materials[light_instance_index];
      li.wi        = wi;
      li.normal    = light_normal;
      li.dist_sq   = dist_sq;
      li.cos_theta = cos_theta;
      li.cos_light = cos_light;
      li.pdf_area  = discrete_probability * light_record.inv_area;
      return li;
    }
  }
  return {};
}

inline math::vec3 sample_light(const scene_t    &scene,     //
                               const bvh::bvh_t &tlas,      //
                               const math::vec3 &hit_pos,   //
                               const math::vec3 &normal,    //
                               const math::vec3 &wo,        //
                               const material_t &material,  //
                               sampler_t        &sampler,   //
                               const math::vec2 &uv) {
  light_intersection_t li = find_visible_light(scene, tlas, hit_pos, normal, wo,
                                               material, sampler, uv);
  if (li.is_valid) {
    math::vec3 emission = li.material.emitted(sampler, -li.wi, li.normal, uv);
    math::vec3 brdf     = material.evaluate(sampler, li.wi, wo, normal, uv);
    float      G        = (li.cos_theta * li.cos_light) / li.dist_sq;
    float      pdf      = li.pdf_area;
    return (brdf * emission * G) / pdf;
  }
  return math::vec3{0.f};
}

struct light_sample_t {
  math::vec3 radiance;
  float      pdf;
};

inline light_sample_t sample_light_mis(const scene_t    &scene,     //
                                       const bvh::bvh_t &tlas,      //
                                       const math::vec3 &hit_pos,   //
                                       const math::vec3 &normal,    //
                                       const math::vec3 &wo,        //
                                       const material_t &material,  //
                                       sampler_t        &sampler,
                                       const math::vec2 &uv) {
  light_intersection_t li = find_visible_light(scene, tlas, hit_pos, normal, wo,
                                               material, sampler, uv);
  if (li.is_valid) {
    math::vec3 emission  = li.material.emitted(sampler, -li.wi, li.normal, uv);
    math::vec3 brdf      = material.evaluate(sampler, li.wi, wo, normal, uv);
    float      G         = (li.cos_theta * li.cos_light) / li.dist_sq;
    float      pdf_area  = li.pdf_area;
    float      pdf_light = pdf_area * (li.dist_sq / li.cos_light);
    float      pdf_brdf  = material.pdf(sampler, li.wi, wo, normal, uv);
    float      weight    = power_huristic(pdf_light, pdf_brdf);
    math::vec3 radiance  = (brdf * emission * G * weight) / pdf_area;
    return {radiance, pdf_light};
  }
  return {{}, 0.f};
}

inline float calculate_light_pdf(const scene_t     &scene,  //
                                 const math::vec3  &hit_pos,
                                 const tlas::hit_t &light_hit,
                                 const math::vec3  &ray_dir) {
  const auto &instance = scene.instances[light_hit.instance_index];
  uint32_t    record_index =
      scene.instance_light_maps[light_hit.instance_index]
          .triangle_to_light_record[light_hit.blas_hit.prim_index];
  if (record_index == std::numeric_limits<uint32_t>::max()) return 0.f;
  const auto &record       = scene.light_records[record_index];
  const auto &blas         = scene.blases[instance.blas_index];
  const auto &triangle     = blas.triangles[light_hit.blas_hit.prim_index];
  float       pdf_area     = record.probability * record.inv_area;
  float       dist_sq      = light_hit.blas_hit.t * light_hit.blas_hit.t;
  math::vec3  normal_light = get_world_normal(scene, instance, light_hit);
  float       cos_light    = math::dot(normal_light, -ray_dir);
  if (cos_light <= 0.0f) return 0.0f;
  return pdf_area * (dist_sq / cos_light);
}

// inline math::vec3 naive_Li(const scene_t    &scene,  //
//                            const bvh::bvh_t &tlas,   //
//                            bvh::ray_t        ray,    //
//                            bool              nee,    //
//                            bool              mis,    //
//                            sampler_t        &sampler) {
//   math::vec3 throughput{1.0f}, radiance{0.0f};
//   material_t prev_mat;
//   prev_mat.type            = material_type_t::e_unknown;
//   float      last_brdf_pdf = 0.f;
//   math::vec3 last_hit_pos  = ray.origin;
//   math::vec2 last_uv{0.f, 0.f};
//
//   // number of bounces doesnt matter since I do rr based termination
//   for (uint32_t bounce = 0; bounce < 1000u; bounce++) {
//     auto hit =
//         tlas::intersect_tlas(tlas.nodes.data(), tlas.prim_indices.data(),
//                              scene.instances.data(), scene.blases.data(),
//                              ray);
//
//     if (!hit.did_intersect()) {
//       radiance += throughput * scene.background(ray);
//       break;
//     }
//
//     const auto     &instance   = scene.instances[hit.instance_index];
//     const auto     &material   = scene.materials[hit.instance_index];
//     model::vertex_t hit_vertex = interpolate_vertex(scene, instance, hit);
//     math::vec3      hit_pos    = ray.origin + ray.direction * hit.blas_hit.t;
//     math::vec3      wo         = -ray.direction;
//
//     if (material.type == material_type_t::e_light) {
//       radiance += throughput * material.emitted(sampler, wo,
//       hit_vertex.normal,
//                                                 hit_vertex.uv);
//       break;
//     }
//
//     auto [should_continue, scatter_sample] =
//         material.sample(sampler, wo, hit_vertex.normal, hit_vertex.uv);
//
//     if (!should_continue) break;
//
//     throughput *= scatter_sample.attenuation;
//
//     if (bounce > 2 && russian_roulette_terminate(sampler, throughput)) break;
//
//     math::vec3 offset_normal =
//         math::dot(scatter_sample.wi, hit_vertex.normal) > 0.f
//             ? hit_vertex.normal
//             : -hit_vertex.normal;
//     ray           = bvh::ray_t::create(hit_pos + offset_normal * epsilon,
//                                        scatter_sample.wi);
//     last_brdf_pdf = scatter_sample.pdf;
//     last_hit_pos  = hit_pos;
//     last_uv       = hit_vertex.uv;
//     prev_mat      = material;
//   }
//   return radiance;
// }

// inline math::vec3 nee_Li(const scene_t    &scene,  //
//                          const bvh::bvh_t &tlas,   //
//                          bvh::ray_t        ray,    //
//                          bool              nee,    //
//                          bool              mis,    //
//                          sampler_t        &sampler) {
//   math::vec3 throughput{1.0f}, radiance{0.0f};
//   material_t prev_mat;
//   prev_mat.type            = material_type_t::e_unknown;
//   float      last_brdf_pdf = 0.f;
//   math::vec3 last_hit_pos  = ray.origin;
//   math::vec2 last_uv{0.f, 0.f};
//
//   // number of bounces doesnt matter since I do rr based termination
//   for (uint32_t bounce = 0; bounce < 1000u; bounce++) {
//     auto hit =
//         tlas::intersect_tlas(tlas.nodes.data(), tlas.prim_indices.data(),
//                              scene.instances.data(), scene.blases.data(),
//                              ray);
//
//     if (!hit.did_intersect()) {
//       radiance += throughput * scene.background(ray);
//       break;
//     }
//
//     const auto     &instance   = scene.instances[hit.instance_index];
//     const auto     &material   = scene.materials[hit.instance_index];
//     model::vertex_t hit_vertex = interpolate_vertex(scene, instance, hit);
//     math::vec3      hit_pos    = ray.origin + ray.direction * hit.blas_hit.t;
//     math::vec3      wo         = -ray.direction;
//
//     if (material.type == material_type_t::e_light) {
//       radiance += throughput * material.emitted(sampler, wo,
//       hit_vertex.normal,
//                                                 hit_vertex.uv);
//       break;
//     }
//
//     auto [should_continue, scatter_sample] =
//         material.sample(sampler, wo, hit_vertex.normal, hit_vertex.uv);
//
//     if (!should_continue) break;
//
//     throughput *= scatter_sample.attenuation;
//
//     if (bounce > 2 && russian_roulette_terminate(sampler, throughput)) break;
//
//     math::vec3 offset_normal =
//         math::dot(scatter_sample.wi, hit_vertex.normal) > 0.f
//             ? hit_vertex.normal
//             : -hit_vertex.normal;
//     ray           = bvh::ray_t::create(hit_pos + offset_normal * epsilon,
//                                        scatter_sample.wi);
//     last_brdf_pdf = scatter_sample.pdf;
//     last_hit_pos  = hit_pos;
//     last_uv       = hit_vertex.uv;
//     prev_mat      = material;
//   }
//   return radiance;
// }

inline math::vec3 trace_path(const scene_t    &scene,  //
                             const bvh::bvh_t &tlas,   //
                             bvh::ray_t        ray,    //
                             bool              nee,    //
                             bool              mis,    //
                             sampler_t        &sampler) {
  math::vec3 throughput{1.0f}, radiance{0.0f};
  material_t prev_mat;
  prev_mat.type            = material_type_t::e_unknown;
  float      last_brdf_pdf = 0.f;
  math::vec3 last_hit_pos  = ray.origin;
  math::vec2 last_uv{0.f, 0.f};

  // number of bounces doesnt matter since I do rr based termination
  for (uint32_t bounce = 0; bounce < 10000u; bounce++) {
    auto hit =
        tlas::intersect_tlas(tlas.nodes.data(), tlas.prim_indices.data(),
                             scene.instances.data(), scene.blases.data(), ray);

    if (!hit.did_intersect()) {
      radiance += throughput * scene.background(ray);
      break;
    }

    const auto     &instance   = scene.instances[hit.instance_index];
    const auto     &material   = scene.materials[hit.instance_index];
    model::vertex_t hit_vertex = interpolate_vertex(scene, instance, hit);
    math::vec3      hit_pos    = ray.origin + ray.direction * hit.blas_hit.t;
    math::vec3      wo         = -ray.direction;

    if (material.type == material_type_t::e_light) {
      float weight = 1.f;
      if (!nee) {
        // path tracing: no need to do anything, weight stays 1.f
      } else if (!mis) {
        // pure nee
        if (bounce > 0 && !prev_mat.is_specular()) {
          weight = 0.f;
        }
      } else {
        // mis
        if (bounce > 0 && !prev_mat.is_specular()) {
          float pdf_light =
              calculate_light_pdf(scene, last_hit_pos, hit, ray.direction);
          float pdf_brdf = last_brdf_pdf;
          weight         = power_huristic(pdf_brdf, pdf_light);
        }
      }

      radiance +=
          throughput *
          material.emitted(sampler, wo, hit_vertex.normal, hit_vertex.uv) *
          weight;
      break;
    }

    auto [should_continue, scatter_sample] =
        material.sample(sampler, wo, hit_vertex.normal, hit_vertex.uv);

    if (nee && !material.is_specular()) {
      if (!mis) {
        // pure nee
        radiance +=
            throughput * sample_light(scene, tlas, hit_pos, hit_vertex.normal,
                                      wo, material, sampler, hit_vertex.uv);
      } else {
        light_sample_t light_sample =
            sample_light_mis(scene, tlas, hit_pos, hit_vertex.normal, wo,
                             material, sampler, hit_vertex.uv);
        radiance += throughput * light_sample.radiance;
      }
    }
    if (!should_continue) break;

    throughput *= scatter_sample.attenuation;

    if (bounce > 2 && russian_roulette_terminate(sampler, throughput)) break;

    math::vec3 offset_normal =
        math::dot(scatter_sample.wi, hit_vertex.normal) > 0.f
            ? hit_vertex.normal
            : -hit_vertex.normal;
    ray           = bvh::ray_t::create(hit_pos + offset_normal * epsilon,
                                       scatter_sample.wi);
    last_brdf_pdf = scatter_sample.pdf;
    last_hit_pos  = hit_pos;
    last_uv       = hit_vertex.uv;
    prev_mat      = material;
  }
  return radiance;
}

#endif
