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
#include "config.hpp"
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
#include "utility.hpp"
#include "integrator.hpp"

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

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0]
              << " <config_file.yaml> <output_image_name>\n";
    return EXIT_FAILURE;
  }

  config_t config;
  try {
    config = load_config(argv[1]);
  } catch (const std::exception &e) {
    std::cerr << "Fatal Error loading config: " << e.what() << '\n';
    return EXIT_FAILURE;
  }

  scene_t scene{};

  math::vec3 bg_color = config.background;
  scene.background    = [bg_color](const bvh::ray_t &ray) -> math::vec3 {
    return bg_color;
  };

  for (const auto &entity : config.entities) {
    auto model = model::load_model_from_path(entity.path);

    for (const auto &mesh : model.meshes) {
      std::cout << "Loading mesh: " << mesh.name << '\n';
      auto triangles = model::create_triangles_from_mesh(mesh);
      auto aabbs     = math::aabbs_from_triangles(triangles);
      scene.blases.emplace_back(bvh::build_bvh_sweep_sah(aabbs), triangles);
      uint32_t mesh_index = static_cast<uint32_t>(scene.raw_meshes.size());
      scene.raw_meshes.push_back(mesh);
      if (entity.material.type == material_type_t::e_lambertian) {
        auto diffuse_tex = get_diffuse_texture(mesh.material_description);
        add_instance(scene, entity.transform, create_lambertian(diffuse_tex),
                     mesh_index);
      } else if (entity.material.type == material_type_t::e_light) {
        add_instance(scene, entity.transform,
                     create_light(entity.material.as.light.emission),
                     mesh_index);
      }
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

  camera_t &camera = config.camera;

  image_t image{640, 640};
  // image_t  image{1920, 1200};
  config.camera.set_dimensions(image.width, image.height);

  auto start = std::chrono::high_resolution_clock::now();

  render(16, config.max_spp, 4, image, argv[2],
         [&](uint32_t x, uint32_t y, uint32_t current_spp) -> math::vec3 {
           sampler_t sampler = create_white_noise_sampler(
               x, y, image.width, image.height, current_spp);
           float jitter_x = static_cast<float>(x) + (sampler.randf() - 0.5f);
           float jitter_y = static_cast<float>(y) + (sampler.randf() - 0.5f);

           auto [O, D]    = camera.ray_gen(jitter_x, jitter_y);
           bvh::ray_t ray = bvh::ray_t::create(O, D);

           math::vec3 color =
               trace_path(scene, tlas, ray, config.nee, config.mis, sampler);

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
  for (int j = image.height - 1; j >= 0; j--)
    for (int i = 0; i < image.width; i++) {
      sum += image.at(i, j);
    }
  std::cout << "pixel average: " << sum / float(image.width * image.height)
            << '\n';

  return 0;
}
