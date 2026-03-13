#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
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
#include "random.hpp"
#include "render.hpp"
#include "scene.hpp"

math::mat4 create_transform(math::vec3 translation, math::vec3 rotation,
                            math::vec3 scale) {
  return math::translate(math::mat4{1.f}, translation) *
         math::toMat4(math::quat{rotation}) *
         math::scale(math::mat4{1.f}, scale);
}

void add_instance(scene_t         &scene,
                  const math::mat4 transform,  //
                  const material_t material) {
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
}

bool russian_roulette_terminate(random_t &random, math::vec3 &throughput) {
  float p =
      std::min(1.0f, std::max({throughput.r, throughput.g, throughput.b}));
  if (random.randf() > p) return true;
  throughput /= p;
  return false;
}

// TODO: make bvh traversal const
// TODO: make tlas traversal const
// TODO: make scene const
math::vec3 sample_light(scene_t &scene, bvh::bvh_t &tlas,
                        const math::vec3 &hit_pos, const math::vec3 &normal,
                        const scatter_sample_t &scatter_sample, bvh::ray_t ray,
                        random_t &random) {
  if (!scene.light_instance_indices.size()) return math::vec3{0.f};
  uint32_t    light_instance_index = scene.light_instance_indices[random.sample(
      scene.light_instance_indices.size())];
  const auto &light_instance       = scene.instances[light_instance_index];
  const auto &light_blas           = scene.blases[light_instance.blas_index];
  uint32_t    triangle_index       = random.sample(light_blas.triangles.size());
  auto        light_triangle       = light_blas.triangles[triangle_index];
  light_triangle.v0 =
      light_instance.transform * math::vec4{light_triangle.v0, 1.f};
  light_triangle.v1 =
      light_instance.transform * math::vec4{light_triangle.v1, 1.f};
  light_triangle.v2 =
      light_instance.transform * math::vec4{light_triangle.v2, 1.f};

  math::vec3 p_world      = random.sample_triangle(light_triangle);
  math::vec3 light_normal = light_triangle.normal();

  math::vec3 L       = p_world - hit_pos;
  float      dist_sq = math::length2(L);
  float      dist    = std::sqrt(dist_sq);
  L /= dist;

  float cos_theta = math::dot(normal, L);
  float cos_light = math::dot(light_normal, -L);

  if (cos_theta > 0.0f && cos_light > 0.0f) {
    bvh::ray_t shadow_ray = bvh::ray_t::create(hit_pos + normal * epsilon, L);
    shadow_ray.tmin       = 0;
    shadow_ray.tmax       = dist - (epsilon * 10.f);
    auto shadow_hit       = tlas::intersect_tlas(
        tlas.nodes.data(), tlas.prim_indices.data(), scene.instances.data(),
        scene.blases.data(), shadow_ray);

    if (!shadow_hit.did_intersect()) {
      float G = (cos_theta * cos_light) / dist_sq;
      float light_selection_probability =
          1.0f /
          (light_blas.triangles.size() * scene.light_instance_indices.size());
      float            area = light_triangle.area();
      const math::vec3 emission =
          scene.materials[light_instance_index].as.light.emission;
      return (scatter_sample.brdf * emission * G * area /
              light_selection_probability);
    }
  }
  return math::vec3{0.f};
}

constexpr uint32_t max_bounce = 100;

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

bool is_specular(material_t material) {
  return material.type == material_type_t::e_metal ||
         material.type == material_type_t::e_dielectric;
}

// TODO: make bvh traversal const
// TODO: make tlas traversal const
// TODO: make scene const
math::vec3 trace_path(scene_t &scene, bvh::bvh_t &tlas, bvh::ray_t ray,
                      bool nee, random_t &random) {
  math::vec3 throughput{1.0f}, color{0.0f};
  material_t prev_mat{.type = material_type_t::e_unknown};

  for (uint32_t bounce = 0; bounce < max_bounce; bounce++) {
    auto hit =
        tlas::intersect_tlas(tlas.nodes.data(), tlas.prim_indices.data(),
                             scene.instances.data(), scene.blases.data(), ray);

    if (!hit.did_intersect()) {
      color += throughput * scene.background(ray);
      break;
    }

    const auto &instance = scene.instances[hit.instance_index];
    const auto &material = scene.materials[hit.instance_index];
    math::vec3  normal   = get_world_normal(scene, instance, hit);
    math::vec3  hit_pos  = ray.origin + ray.direction * hit.blas_hit.t;

    if (material.type == material_type_t::e_light) {
      if (math::dot(normal, ray.direction) < 0.0f) {
        if (!nee || bounce == 0 || is_specular(prev_mat)) {
          color += throughput * material.as.light.emission;
        }
      }
      break;
    }

    scatter_sample_t scatter_sample =
        sample(material, ray, hit_pos, normal, random);

    if (nee && material.type == material_type_t::e_lambertian) {
      color += throughput * sample_light(scene, tlas, hit_pos, normal,
                                         scatter_sample, ray, random);
    }

    if (russian_roulette_terminate(random, throughput)) break;

    if (!scatter_sample.active) break;
    ray = scatter_sample.outgoing_ray;
    throughput *=
        scatter_sample.brdf * scatter_sample.cosine / scatter_sample.pdf;
    prev_mat = material;
  }
  return color;
}

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cerr << "Usage: [pathtracing] [model] [name] [nee] [spp]\n";
    exit(EXIT_FAILURE);
  }

  bool     nee     = std::stoi(argv[3]);
  uint32_t max_spp = std::stoi(argv[4]);

  scene_t scene{};
  scene.background = [](const bvh::ray_t &ray) { return math::vec3{0}; };

  const math::vec3 white{.73, .73, .73};
  const math::vec3 red{.65, .05, .05};
  const math::vec3 green{.12, .45, .15};
  const float      pi = math::pi<float>();

  camera_t camera{90.f, {0, 1, 1.75}, {0, 1, 0}};

  {
    auto model = model::load_model_from_path(argv[1]);
    for (auto &mesh : model.meshes) {
      auto          triangles = model::create_triangles_from_mesh(mesh);
      auto          aabbs     = math::aabbs_from_triangles(triangles);
      tlas::blas_t &blas      = scene.blases.emplace_back(
          bvh::build_bvh_sweep_sah(aabbs), std::move(triangles));

      bool is_emissive = std::ranges::any_of(
          mesh.material_description.texture_infos,
          [](model::texture_info_t info) {
            return info.texture_type == model::texture_type_t::e_emissive_color;
          });
      if (is_emissive) {
        auto emissive =
            *std::find_if(mesh.material_description.texture_infos.begin(),
                          mesh.material_description.texture_infos.end(),
                          [](model::texture_info_t info) {
                            return info.texture_type ==
                                   model::texture_type_t::e_emissive_color;
                          });
        if (emissive.emissive_color == math::vec3{0}) is_emissive = false;
      }

      auto transform = create_transform({0, 0, 0}, {0, 0, 0}, {1, 1, 1});
      std::cout << mesh.name << ' ' << is_emissive << '\n';
      if (is_emissive) {
        auto emissive =
            *std::find_if(mesh.material_description.texture_infos.begin(),
                          mesh.material_description.texture_infos.end(),
                          [](model::texture_info_t info) {
                            return info.texture_type ==
                                   model::texture_type_t::e_emissive_color;
                          });
        // add_instance(scene, transform,
        // create_light(emissive.emissive_color));
        add_instance(scene, transform, create_lambertian(math::vec3{1.f}));
      } else {
        auto diffuse =
            *std::find_if(mesh.material_description.texture_infos.begin(),
                          mesh.material_description.texture_infos.end(),
                          [](model::texture_info_t info) {
                            return info.texture_type ==
                                   model::texture_type_t::e_diffuse_color;
                          });
        add_instance(scene, transform,
                     create_lambertian(diffuse.diffuse_color));
      }
    }
  }
  {
    auto model = model::load_model_from_path("./sphere.obj");
    for (auto &mesh : model.meshes) {
      auto          triangles = model::create_triangles_from_mesh(mesh);
      auto          aabbs     = math::aabbs_from_triangles(triangles);
      tlas::blas_t &blas      = scene.blases.emplace_back(
          bvh::build_bvh_sweep_sah(aabbs), std::move(triangles));
      add_instance(scene,
                   create_transform({0, 1, 0}, {0, 0, 0}, {0.1, 0.1, 0.1}),
                   create_light(math::vec3{20}));
    }
  }

  bvh::bvh_t tlas = bvh::build_bvh_sweep_sah(scene.instance_aabbs);

  for (uint32_t i = 0; i < scene.materials.size(); i++)
    if (scene.materials[i].type == material_type_t::e_light)
      scene.light_instance_indices.push_back(i);

  image_t image{640, 640};
  // image_t  image{1920, 1200};
  camera.set_dimentions(image._width, image._height);

  auto start = std::chrono::high_resolution_clock::now();

  render(16, max_spp, 4, image, argv[2],
         [&](uint32_t x, uint32_t y, random_t &rng) -> math::vec3 {
           float jitter_x = static_cast<float>(x) + (rng.randf() - 0.5f);
           float jitter_y = static_cast<float>(y) + (rng.randf() - 0.5f);

           auto [O, D]    = camera.ray_gen(jitter_x, jitter_y);
           bvh::ray_t ray = bvh::ray_t::create(O, D);

           math::vec3 color = trace_path(scene, tlas, ray, nee, rng);

           if (math::any(math::isnan(color))) throw std::runtime_error("nan");

           return math::any(math::isnan(color)) ? math::vec3{0.f} : color;
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
