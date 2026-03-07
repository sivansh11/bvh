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

math::vec3 random_color_from_hit(uint32_t v) {
  return {(((v + 1) * 123) % 255) / 255.f, (((v + 1) * 456) % 255) / 255.f,
          (((v + 1) * 789) % 255) / 255.f};
}

math::vec4 turbo_color_map(float x) {
  const math::vec4 kRedVec4 =
      math::vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
  const math::vec4 kGreenVec4 =
      math::vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
  const math::vec4 kBlueVec4 =
      math::vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
  const math::vec2 kRedVec2   = math::vec2(-152.94239396, 59.28637943);
  const math::vec2 kGreenVec2 = math::vec2(4.27729857, 2.82956604);
  const math::vec2 kBlueVec2  = math::vec2(-89.90310912, 27.34824973);

  x             = clamp(x, 0, 1);
  math::vec4 v4 = math::vec4(1.0, x, x * x, x * x * x);
  math::vec2 v2 = math::vec2{v4.z, v4.w} * v4.z;
  return math::vec4(dot(v4, kRedVec4) + dot(v2, kRedVec2),
                    dot(v4, kGreenVec4) + dot(v2, kGreenVec2),
                    dot(v4, kBlueVec4) + dot(v2, kBlueVec2), 1);
}

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

math::mat4 get_model_fit_transform(const std::vector<tlas::blas_t> &blases,
                                   uint32_t                         blas_index,
                                   math::vec3 target_position,
                                   math::vec3 rotation, float target_size) {
  math::aabb_t original_aabb = blases[blas_index].bvh.nodes[0].aabb();
  math::vec3   size          = original_aabb.max - original_aabb.min;
  math::vec3   center        = (original_aabb.max + original_aabb.min) * 0.5f;
  float        max_dim       = std::max({size.x, size.y, size.z});
  float        scale_factor  = (max_dim > 0) ? (target_size / max_dim) : 1.0f;
  math::mat4   transform = math::translate(math::mat4(1.0f), target_position);
  transform              = transform * math::toMat4(math::quat(rotation));
  transform              = math::scale(transform, math::vec3(scale_factor));
  transform              = math::translate(transform, -center);

  return transform;
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
                        const material_t &material, bvh::ray_t ray,
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
    bvh::ray_t shadow_ray = bvh::ray_t::create(hit_pos, L);
    shadow_ray.tmin       = epsilon;
    shadow_ray.tmax       = dist - epsilon;
    auto shadow_hit       = tlas::intersect_tlas(
        tlas.nodes.data(), tlas.prim_indices.data(), scene.instances.data(),
        scene.blases.data(), shadow_ray);

    if (!shadow_hit.did_intersect()) {
      float G = (cos_theta * cos_light) / dist_sq;
      float light_selection_probability =
          1.0f /
          (light_blas.triangles.size() * scene.light_instance_indices.size());
      float area = light_triangle.area();
      math::vec3 brdf = material.as.lambertian.albedo / math::pi<float>();
      return (brdf * scene.materials[light_instance_index].as.light.emission *
              G * area / light_selection_probability);
    }
  }
  return math::vec3{0.f};
}

constexpr uint32_t max_spp    = 2048;
constexpr uint32_t max_bounce = 100;
constexpr bool     nee        = true;

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
                      random_t &random) {
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

    if (nee && material.type == material_type_t::e_lambertian) {
      color += throughput * sample_light(scene, tlas, hit_pos, normal, material,
                                         ray, random);
    }

    if (russian_roulette_terminate(random, throughput)) break;

    brdf_t brdf = sample(material, ray, hit_pos, normal, random);
    if (!brdf.active) break;
    ray = brdf.outgoing_ray;
    throughput *= brdf.value / brdf.pdf;
    prev_mat = material;
  }
  return color;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: [pathtracing] [model] [name]\n";
    exit(EXIT_FAILURE);
  }

  scene_t scene{};
  scene.background = [](const bvh::ray_t &ray) { return math::vec3{0}; };

  const math::vec3 white{.73, .73, .73};
  const math::vec3 red{.65, .05, .05};
  const math::vec3 green{.12, .45, .15};
  const float      pi = math::pi<float>();

  {
    auto model = model::load_model_from_path("./plain.obj");
    for (auto &mesh : model.meshes) {
      auto          triangles = model::create_triangles_from_mesh(mesh);
      auto          aabbs     = math::aabbs_from_triangles(triangles);
      tlas::blas_t &blas      = scene.blases.emplace_back(
          bvh::build_bvh_sweep_sah(aabbs), std::move(triangles));
    }

    // light
    add_instance(  //
        scene,
        create_transform({0.f, 0.49f, 0.f},  //
                         {pi, 0.f, 0.f},     //
                         {0.3f, 0.3f, 0.3f}),
        create_light(math::vec3{15.f}));

    // bottom floor
    add_instance(  //
        scene,
        create_transform({0.f, -0.5f, 0.f},  //
                         {0.f, 0.f, 0.f},    //
                         {1.f, 1.f, 1.f}),
        create_lambertian(white));
    // top floor
    add_instance(  //
        scene,
        create_transform({0.f, 0.5f, 0.f},  //
                         {pi, 0.f, 0.f},    //
                         {1.f, 1.f, 1.f}),
        create_lambertian(white));
    // left red floor
    add_instance(  //
        scene,
        create_transform({0.5f, 0.f, 0.f},    //
                         {0.f, 0.f, pi / 2},  //
                         {1.f, 1.f, 1.f}),
        create_lambertian(red));
    // right green floor
    add_instance(  //
        scene,
        create_transform({-0.5f, 0.f, 0.f},    //
                         {0.f, 0.f, -pi / 2},  //
                         {1.f, 1.f, 1.f}),
        create_lambertian(green));
    // back floor
    add_instance(  //
        scene,
        create_transform({0.f, 0.f, 0.5f},     //
                         {-pi / 2, 0.f, 0.f},  //
                         {1.f, 1.f, 1.f}),
        create_lambertian(white));

    add_instance(  //
        scene,
        create_transform({0.f, 0.f, -1.5f},   //
                         {pi / 2, 0.f, 0.f},  //
                         {1.f, 1.f, 1.f}),
        create_light(math::vec3{3}));
  }

  // user model
  {
    auto model = model::load_model_from_path(argv[1]);
    for (auto &mesh : model.meshes) {
      auto          triangles = model::create_triangles_from_mesh(mesh);
      auto          aabbs     = math::aabbs_from_triangles(triangles);
      tlas::blas_t &blas      = scene.blases.emplace_back(
          bvh::build_bvh_sweep_sah(aabbs), std::move(triangles));
    }
    uint32_t   blas_index = scene.blases.size() - 1;
    math::mat4 transform  = get_model_fit_transform(scene.blases,         //
                                                    blas_index,           //
                                                    {0.f, -0.2f, -0.2f},  //
                                                    {0.f, pi / 2, 0.f},   //
                                                    0.6f);

    add_instance(  //
        scene,
        transform,  //
        create_lambertian(math::vec3{0.9}));
  }

  bvh::bvh_t tlas = bvh::build_bvh_sweep_sah(scene.instance_aabbs);

  for (uint32_t i = 0; i < scene.materials.size(); i++)
    if (scene.materials[i].type == material_type_t::e_light)
      scene.light_instance_indices.push_back(i);

  image_t image{640, 640};
  // image_t  image{1920, 1200};
  camera_t camera{90.f, {0.f, 0.f, -1.025f}, {0, 0, 0}};
  camera.set_dimentions(image._width, image._height);

  auto start = std::chrono::high_resolution_clock::now();

  render(16, max_spp, 4, image, argv[2],
         [&](uint32_t x, uint32_t y, random_t &rng) -> math::vec3 {
           float jitter_x = static_cast<float>(x) + (rng.randf() - 0.5f);
           float jitter_y = static_cast<float>(y) + (rng.randf() - 0.5f);

           auto [O, D]    = camera.ray_gen(jitter_x, jitter_y);
           bvh::ray_t ray = bvh::ray_t::create(O, D);

           math::vec3 color = trace_path(scene, tlas, ray, rng);

           return math::any(math::isnan(color)) ? math::vec3{0.f} : color;
         });

  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "render took: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << "\n";  //

  return 0;
}
