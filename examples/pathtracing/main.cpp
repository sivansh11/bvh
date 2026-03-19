#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include "bvh/bvh.hpp"
#include "bvh/tlas.hpp"
#include "bvh/traversal.hpp"
#include "camera.hpp"
#include "common.hpp"
#include "glm/common.hpp"
#include "glm/fwd.hpp"
#include "glm/geometric.hpp"
#include "glm/vector_relational.hpp"
#include "image.hpp"
#include "material.hpp"
#include "math/aabb.hpp"
#include "math/math.hpp"
#include "math/triangle.hpp"
#include "math/utilies.hpp"
#include "model/model.hpp"
#include "render.hpp"
#include "sampler.hpp"
#include "scene.hpp"

math::mat4 create_transform(math::vec3 translation,  //
                            math::vec3 rotation,     //
                            math::vec3 scale) {
  return math::translate(math::mat4{1.f}, translation) *
         math::toMat4(math::quat{rotation}) *
         math::scale(math::mat4{1.f}, scale);
}

void add_instance(scene_t         &scene,
                  const math::mat4 transform,  //
                  const material_t material, uint32_t mesh_index) {
  uint32_t     blas_index = scene.blases.size() - 1;
  math::aabb_t blas_aabb  = scene.blases[blas_index].bvh.nodes[0].aabb();
  math::aabb_t transformed_aabb{};
  for (int i = 0; i < 8; i++) {
    math::vec3 corner = {
        (i & 1) ? blas_aabb.max.x : blas_aabb.min.x,
        (i & 2) ? blas_aabb.max.y : blas_aabb.min.y,
        (i & 4) ? blas_aabb.max.z : blas_aabb.min.z,
    };
    math::vec3 transformed_corner = transform * math::vec4{corner, 1};
    transformed_aabb.grow(transformed_corner);
  }
  scene.instances.emplace_back(transform, math::inverse(transform),
                               transformed_aabb, blas_index);
  scene.instance_aabbs.push_back(transformed_aabb);
  scene.materials.emplace_back(material);
  scene.instance_to_mesh_index.push_back(mesh_index);
}

bool russian_roulette_terminate(sampler_t &sampler, math::vec3 &throughput) {
  float p =
      std::min(1.0f, std::max({throughput.r, throughput.g, throughput.b}));
  if (sampler.randf() > p) return true;
  throughput /= p;
  return false;
}

// TODO: make bvh traversal const
// TODO: make tlas traversal const
// TODO: make scene const
math::vec3 sample_light(scene_t          &scene,     //
                        bvh::bvh_t       &tlas,      //
                        const math::vec3 &hit_pos,   //
                        const math::vec3 &normal,    //
                        const math::vec3 &wo,        //
                        const material_t &material,  //
                        sampler_t &sampler, const math::vec2 &uv) {
  if (!scene.light_instance_indices.size()) return math::vec3{0.f};

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

  math::vec3 p_world      = sampler.sample_triangle(triangle);
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
      math::vec3 emission = scene.materials[light_instance_index].emitted(
          sampler, -wi, light_normal, uv);
      math::vec3 brdf = material.evaluate(sampler, wi, wo, normal, uv);
      float      G    = (cos_theta * cos_light) / dist_sq;
      float      pdf  = discrete_probability * light_record.inv_area;
      return (brdf * emission * G) / pdf;
    }
  }
  return math::vec3{0.f};
}

struct light_sample_t {
  math::vec3 radiance;
  float      pdf;
};

float power_huristic(float a, float b) {
  if (math::isinf(a) || math::isnan(a)) return 0.5f;
  if (math::isinf(b) || math::isnan(b)) return 0.5f;
  // TODO: maybe throw ?
  if (a == 0.f && b == 0.f) return 0.5;
  float a2 = a * a;
  float b2 = b * b;
  return a2 / (a2 + b2);
}

light_sample_t sample_light_mis(scene_t          &scene,     //
                                bvh::bvh_t       &tlas,      //
                                const math::vec3 &hit_pos,   //
                                const math::vec3 &normal,    //
                                const math::vec3 &wo,        //
                                const material_t &material,  //
                                sampler_t &sampler, const math::vec2 &uv) {
  if (!scene.light_instance_indices.size()) return {{}, 0.f};

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

  math::vec3 p_world      = sampler.sample_triangle(triangle);
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
      math::vec3 emission = scene.materials[light_instance_index].emitted(
          sampler, -wi, light_normal, uv);
      math::vec3 brdf      = material.evaluate(sampler, wi, wo, normal, uv);
      float      G         = (cos_theta * cos_light) / dist_sq;
      float      pdf_area  = discrete_probability * light_record.inv_area;
      float      pdf_light = pdf_area * (dist_sq / cos_light);
      float      pdf_brdf  = material.pdf(sampler, wi, wo, normal, uv);
      float      weight    = power_huristic(pdf_light, pdf_brdf);
      math::vec3 radiance  = (brdf * emission * G * weight) / pdf_area;
      return {radiance, pdf_light};
    }
  }
  return {{}, 0.f};
}

math::vec3 get_world_normal(const scene_t          &scene,
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

// TODO: make bvh traversal const
// TODO: make tlas traversal const
// TODO: make scene const
float calculate_light_pdf(const scene_t     &scene,  //
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

// TODO: make bvh traversal const
// TODO: make tlas traversal const
// TODO: make scene const
math::vec3 trace_path(scene_t    &scene,  //
                      bvh::bvh_t &tlas,   //
                      bvh::ray_t  ray,    //
                      bool        nee,    //
                      bool        mis,    //
                      sampler_t  &sampler) {
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

    if (russian_roulette_terminate(sampler, throughput)) break;

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

int main(int argc, char **argv) {
  if (argc != 6) {
    std::cerr << "Usage: [pathtracing] [model] [name] [nee] [mis] [spp]\n";
    exit(EXIT_FAILURE);
  }

  bool     nee     = std::stoi(argv[3]);
  bool     mis     = std::stoi(argv[4]);
  uint32_t max_spp = std::stoi(argv[5]);

  scene_t scene{};
  scene.background = [](const bvh::ray_t &ray) { return math::vec3{0}; };

  const math::vec3 white{.73, .73, .73};
  const math::vec3 red{.65, .05, .05};
  const math::vec3 green{.12, .45, .15};
  const float      pi = math::pi<float>();

  math::vec3 from = {750, 600, 500};
  // math::vec3 from = {750, 600, 0};
  // math::vec3 from = {750, 100, 500};
  math::vec3 to = from + math::vec3{-1, 0, 0};
  camera_t   camera{90.f, from, to};
  {
    auto model = model::load_model_from_path("./sphere.obj");
    for (auto &mesh : model.meshes) {
      auto triangles = model::create_triangles_from_mesh(mesh);
      auto aabbs     = math::aabbs_from_triangles(triangles);
      scene.blases.emplace_back(bvh::build_bvh_sweep_sah(aabbs), triangles);
      uint32_t mesh_index = scene.raw_meshes.size();
      scene.raw_meshes.push_back(mesh);
      add_instance(
          scene,
          create_transform({1000, 5000, -750}, {0, 0, 0}, {1000, 1000, 1000}),
          create_light(math::vec3{1000.f}), mesh_index);
    }
  }
  {
    auto model = model::load_model_from_path(argv[1]);
    for (auto &mesh : model.meshes) {
      std::cout << mesh.name << '\n';
      auto triangles = model::create_triangles_from_mesh(mesh);
      auto aabbs     = math::aabbs_from_triangles(triangles);
      scene.blases.emplace_back(bvh::build_bvh_sweep_sah(aabbs), triangles);
      uint32_t mesh_index = scene.raw_meshes.size();
      scene.raw_meshes.push_back(mesh);
      auto diffuse_tex = get_diffuse_texture(mesh.material_description);
      add_instance(scene, create_transform({0, 0, 0}, {0, 0, 0}, {1, 1, 1}),
                   create_lambertian(diffuse_tex), mesh_index);
    }
  }

  bvh::bvh_t tlas = bvh::build_bvh_sweep_sah(scene.instance_aabbs);

  scene.instance_light_maps.resize(scene.instances.size());

  for (uint32_t i = 0; i < scene.materials.size(); i++)
    if (scene.materials[i].type == material_type_t::e_light)
      scene.light_instance_indices.push_back(i);

  float total_power = 0.f;

  for (uint32_t i : scene.light_instance_indices) {
    const auto       &instance  = scene.instances[i];
    const auto       &blas      = scene.blases[instance.blas_index];
    const math::vec3 &emission  = scene.materials[i].as.light.emission;
    const float       luminance = (emission.x + emission.y + emission.z) / 3.f;
    for (const auto &tri : blas.triangles) {
      math::triangle_t world_tri = tri;
      world_tri.v0               = instance.transform * math::vec4{tri.v0, 1.f};
      world_tri.v1               = instance.transform * math::vec4{tri.v1, 1.f};
      world_tri.v2               = instance.transform * math::vec4{tri.v2, 1.f};
      total_power += world_tri.area() * luminance;
    }
  }

  float running_total = 0.f;
  for (uint32_t i : scene.light_instance_indices) {
    const auto       &instance  = scene.instances[i];
    const auto       &blas      = scene.blases[instance.blas_index];
    const math::vec3 &emission  = scene.materials[i].as.light.emission;
    const float       luminance = (emission.x + emission.y + emission.z) / 3.f;
    auto &map = scene.instance_light_maps[i].triangle_to_light_record;
    map.resize(blas.triangles.size(), std::numeric_limits<uint32_t>::max());
    for (uint32_t tri_idx = 0; tri_idx < blas.triangles.size(); tri_idx++) {
      math::triangle_t world_tri = blas.triangles[tri_idx];
      world_tri.v0         = instance.transform * math::vec4{world_tri.v0, 1.f};
      world_tri.v1         = instance.transform * math::vec4{world_tri.v1, 1.f};
      world_tri.v2         = instance.transform * math::vec4{world_tri.v2, 1.f};
      float area           = world_tri.area();
      float triangle_power = area * luminance;
      float triangle_prob =
          (total_power > 0) ? (triangle_power / total_power) : 0.f;
      running_total += triangle_power;
      map[tri_idx] = scene.light_records.size();
      scene.light_records.emplace_back(i, tri_idx, 1.f / area, triangle_prob);
      scene.light_cdfs.push_back(running_total / total_power);
    }
  }

  image_t image{640, 640};
  // image_t  image{1920, 1200};
  camera.set_dimentions(image._width, image._height);

  auto start = std::chrono::high_resolution_clock::now();

  render(16, max_spp, 4, image, argv[2],
         [&](uint32_t x, uint32_t y, uint32_t current_spp) -> math::vec3 {
           sampler_t sampler(x, y, image._width, image._height, current_spp);
           float jitter_x = static_cast<float>(x) + (sampler.randf() - 0.5f);
           float jitter_y = static_cast<float>(y) + (sampler.randf() - 0.5f);

           auto [O, D]    = camera.ray_gen(jitter_x, jitter_y);
           bvh::ray_t ray = bvh::ray_t::create(O, D);

           math::vec3 color = trace_path(scene, tlas, ray, nee, mis, sampler);

           if (math::any(math::isnan(color)) || math::any(math::isinf(color)))
             throw std::runtime_error("something went wrong");

           return color;
         });

  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "render took: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << "\n";  //

  math::vec3 sum{0, 0, 0};
  for (int j = image._height - 1; j >= 0; j--)
    for (int i = 0; i < image._width; i++) {
      sum += image.at(i, j);
    }
  std::cout << "pixel average: " << sum / float(image._width * image._height)
            << '\n';

  return 0;
}
