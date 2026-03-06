#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <thread>
#include <vector>

#include "bvh/bvh.hpp"
#include "bvh/tlas.hpp"
#include "bvh/traversal.hpp"
#include "camera.hpp"
#include "glm/common.hpp"
#include "glm/geometric.hpp"
#include "image.hpp"
#include "math/aabb.hpp"
#include "math/math.hpp"
#include "math/triangle.hpp"
#include "math/utilies.hpp"
#include "model/model.hpp"

math::vec3 random_color_from_hit(uint32_t v) {
  return {(((v + 1) * 123) % 255) / 255.f, (((v + 1) * 456) % 255) / 255.f,
          (((v + 1) * 789) % 255) / 255.f};
}

template <typename fn_t>
void render(uint32_t max_threads, image_t &image, fn_t fn) {
  std::vector<std::pair<uint32_t, uint32_t>> work;
  for (uint32_t y = 0; y < image._height; y++)
    for (uint32_t x = 0; x < image._width; x++)  //
      work.emplace_back(x, y);
  std::vector<std::thread> threads{};
  for (uint32_t thread_index = 0; thread_index < max_threads; thread_index++) {
    threads.emplace_back(
        [&](uint32_t thread_index) {
          for (uint32_t work_index = thread_index; work_index < work.size();
               work_index += max_threads) {
            auto [x, y]                        = work[work_index];
            image.at(x, image._height - y - 1) = fn(x, y);
          }
        },
        thread_index);
  }
  for (auto &thread : threads) thread.join();
}

math::vec4 turbo_color_map(float x) {
  // Source:
  // https://research.google/blog/turbo-an-improved-rainbow-colormap-for-visualization/

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

enum class material_type_t { e_lambertian, e_metal, e_dielectric, e_emissive };

struct lambertian_t {
  math::vec3 albedo;
};

struct metal_t {
  math::vec3 albedo;
  float      fuzz;
};

struct dielectric_t {
  float refraction_index;
};

struct light_t {
  math::vec3 emission;
};

struct material_t {
  material_type_t type;
  union as_t {
    lambertian_t lambertian;
    metal_t      metal;
    dielectric_t dielectric;
    light_t      light;
  } as;
};

material_t create_lambertian(math::vec3 albedo) {
  material_t material;
  material.type                 = material_type_t::e_lambertian;
  material.as.lambertian.albedo = albedo;
  return material;
}

material_t create_metal(math::vec3 albedo, float fuzz) {
  material_t material;
  material.type            = material_type_t::e_metal;
  material.as.metal.albedo = albedo;
  material.as.metal.fuzz   = fuzz;
  return material;
}

material_t create_dielectric(float refraction_index) {
  material_t material;
  material.type                           = material_type_t::e_dielectric;
  material.as.dielectric.refraction_index = refraction_index;
  return material;
}

material_t create_light(math::vec3 emission) {
  material_t material;
  material.type              = material_type_t::e_emissive;
  material.as.light.emission = emission;
  return material;
}

math::mat4 create_transform(math::vec3 translation, math::vec3 rotation,
                            math::vec3 scale) {
  return math::translate(math::mat4{1.f}, translation) *
         math::toMat4(math::quat{rotation}) *
         math::scale(math::mat4{1.f}, scale);
}

void add_instance(const std::vector<tlas::blas_t> &blases,
                  std::vector<tlas::instance_t>   &instances,
                  std::vector<math::aabb_t>       &instance_aabbs,
                  std::vector<material_t>         &instance_materials,
                  const math::mat4                 transform,  //
                  const material_t                 material) {
  uint32_t     blas_index = blases.size() - 1;
  math::aabb_t blas_aabb  = blases[blas_index].bvh.nodes[0].aabb();
  math::aabb_t transformed_aabb{};
  for (int i = 0; i < 8; i++) {
    math::vec3 corner = {
        (i & 1) ? blas_aabb.max.x : blas_aabb.min.x,
        (i & 2) ? blas_aabb.max.y : blas_aabb.min.y,
        (i & 4) ? blas_aabb.max.z : blas_aabb.min.z,
    };
    math::vec3 transformed_corner = transform * math::vec4{corner, 1};
    transformed_aabb.grow(transformed_corner);
    transformed_aabb.min -= 0.001f;
    transformed_aabb.max += 0.001f;
  }
  instances.emplace_back(transform, math::inverse(transform), transformed_aabb,
                         blas_index);
  instance_aabbs.push_back(transformed_aabb);
  instance_materials.emplace_back(material);
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

struct random_t {
  random_t(uint32_t seed) { rng.seed(seed); }
  std::mt19937                          rng{};
  std::uniform_real_distribution<float> dist{0.0, 1.0};

  float randf() { return dist(rng); }

  math::vec3 unit_vector() {
    while (true) {
      auto p     = math::vec3{randf() * 2.f - 1.f, randf() * 2.f - 1.f,
                          randf() * 2.f - 1.f};
      auto lensq = math::length2(p);
      if (1e-160 < lensq && lensq <= 1) return p / math::sqrt(lensq);
    }
  }

  math::vec3 unit_vector_on_hemisphere(const math::vec3 normal) {
    math::vec3 v = unit_vector();
    if (math::dot(v, normal) > 0.0f)
      return v;
    else
      return -v;
  }

  uint32_t sample(uint32_t n) {
    std::uniform_int_distribution<uint32_t> _dist(0, n - 1);
    return _dist(rng);
  }

  math::vec3 sample_triangle(const math::triangle_t &triangle) {
    float u = randf();
    float v = randf();
    if (u + v > 1.f) {
      u = 1.f - u;
      v = 1.f - v;
    }
    return triangle.v0 + u * (triangle.v1 - triangle.v0) +
           v * (triangle.v2 - triangle.v0);
  }
};

bool russian_roulette_terminate(random_t &random, math::vec3 &throughput) {
  float p = std::max({throughput.r, throughput.g, throughput.b});
  if (random.randf() > p) return true;
  throughput /= p;
  return false;
}

math::vec3 background(const bvh::ray_t &ray) {
  return math::vec3{0};
  math::vec3 direction = math::normalize(ray.direction);
  auto       a         = 0.5f * (direction.y + 1.f);
  return (1.f - a) * math::vec3{1} + a * math::vec3{0.5f, 0.7f, 1.f};
}

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: [simple] [model]\n";
    exit(EXIT_FAILURE);
  }

  std::vector<tlas::blas_t>     blases;
  std::vector<tlas::instance_t> instances;
  std::vector<math::aabb_t>     instance_aabbs;
  std::vector<material_t>       materials;

  const math::vec3 white{.73, .73, .73};
  const math::vec3 red{.65, .05, .05};
  const math::vec3 green{.12, .45, .15};
  const float      pi = math::pi<float>();

  {
    auto model = model::load_model_from_path("./plain.obj");
    for (auto &mesh : model.meshes) {
      auto          triangles = model::create_triangles_from_mesh(mesh);
      auto          aabbs     = math::aabbs_from_triangles(triangles);
      tlas::blas_t &blas = blases.emplace_back(bvh::build_bvh_sweep_sah(aabbs),
                                               std::move(triangles));
    }

    // light
    add_instance(                            //
        blases,                              //
        instances,                           //
        instance_aabbs,                      //
        materials,                           //
        create_transform({0.f, 0.49f, 0.f},  //
                         {pi, 0.f, 0.f},     //
                         {0.3f, 0.3f, 0.3f}),
        create_light(math::vec3{15.f}));

    // bottom floor
    add_instance(                            //
        blases,                              //
        instances,                           //
        instance_aabbs,                      //
        materials,                           //
        create_transform({0.f, -0.5f, 0.f},  //
                         {0.f, 0.f, 0.f},    //
                         {1.f, 1.f, 1.f}),
        create_lambertian(white));
    // top floor
    add_instance(                           //
        blases,                             //
        instances,                          //
        instance_aabbs,                     //
        materials,                          //
        create_transform({0.f, 0.5f, 0.f},  //
                         {pi, 0.f, 0.f},    //
                         {1.f, 1.f, 1.f}),
        create_lambertian(white));
    // left red floor
    add_instance(                             //
        blases,                               //
        instances,                            //
        instance_aabbs,                       //
        materials,                            //
        create_transform({0.5f, 0.f, 0.f},    //
                         {0.f, 0.f, pi / 2},  //
                         {1.f, 1.f, 1.f}),
        create_lambertian(red));
    // right green floor
    add_instance(                              //
        blases,                                //
        instances,                             //
        instance_aabbs,                        //
        materials,                             //
        create_transform({-0.5f, 0.f, 0.f},    //
                         {0.f, 0.f, -pi / 2},  //
                         {1.f, 1.f, 1.f}),
        create_lambertian(green));
    // back floor
    add_instance(                              //
        blases,                                //
        instances,                             //
        instance_aabbs,                        //
        materials,                             //
        create_transform({0.f, 0.f, 0.5f},     //
                         {-pi / 2, 0.f, 0.f},  //
                         {1.f, 1.f, 1.f}),
        create_lambertian(white));
  }

  // user model
  {
    auto model = model::load_model_from_path(argv[1]);
    for (auto &mesh : model.meshes) {
      auto          triangles = model::create_triangles_from_mesh(mesh);
      auto          aabbs     = math::aabbs_from_triangles(triangles);
      tlas::blas_t &blas = blases.emplace_back(bvh::build_bvh_sweep_sah(aabbs),
                                               std::move(triangles));
    }
    uint32_t   blas_index = blases.size() - 1;
    math::mat4 transform  = get_model_fit_transform(blases,                //
                                                    blas_index,            //
                                                    {0.f, -0.2f, -0.20f},  //
                                                    {0.f, pi / 2, 0.f},    //
                                                    0.6f);

    add_instance(        //
        blases,          //
        instances,       //
        instance_aabbs,  //
        materials,       //
        transform,       //
        create_metal(math::vec3{1.f}, 0.5f));
  }

  bvh::bvh_t tlas = bvh::build_bvh_sweep_sah(instance_aabbs);

  image_t  image{640, 640};
  camera_t camera{90.f, {0.f, 0.f, -1.025f}, {0, 0, 0}};
  camera.set_dimentions(image._width, image._height);

  auto start = std::chrono::high_resolution_clock::now();

  constexpr uint32_t max_spp    = 64;
  constexpr uint32_t max_bounce = 100;

  render(16, image, [&](uint32_t x, uint32_t y) {
    random_t   random(x + (y * image._width));
    math::vec3 final_color{0.f};

    for (uint32_t s = 0; s < max_spp; s++) {
      float jitter_x = static_cast<float>(x) + (random.randf() - 0.5f);
      float jitter_y = static_cast<float>(y) + (random.randf() - 0.5f);
      auto [O, D]    = camera.ray_gen(jitter_x, jitter_y);
      bvh::ray_t ray = bvh::ray_t::create(O, D);

      math::vec3 throughput{1.0f};
      math::vec3 color{0.0f};

      for (uint32_t bounce = 0; bounce < max_bounce; bounce++) {
        auto hit =
            tlas::intersect_tlas(tlas.nodes.data(), tlas.prim_indices.data(),
                                 instances.data(), blases.data(), ray);
        if (!hit.did_intersect()) {
          color += throughput * background(ray);
          break;
        }

        const auto &instance = instances[hit.instance_index];
        const auto &material = materials[hit.instance_index];

        math::vec3 local_normal = blases[instance.blas_index]
                                      .triangles[hit.blas_hit.prim_index]
                                      .normal();
        math::vec3 normal = math::normalize(
            math::transpose(math::mat3{instance.inv_transform}) * local_normal);
        math::vec3 hit_pos = ray.origin + ray.direction * hit.blas_hit.t;

        if (material.type == material_type_t::e_emissive) {
          color += throughput * material.as.light.emission;
          break;
        }

        if (russian_roulette_terminate(random, throughput)) break;

        switch (material.type) {
          case material_type_t::e_lambertian: {
            math::vec3 scatter_direction =
                random.unit_vector_on_hemisphere(normal);
            ray = bvh::ray_t::create(hit_pos + normal * 0.0001f,
                                     scatter_direction);
            float cos_theta = math::dot(normal, scatter_direction);
            throughput *= material.as.lambertian.albedo * 2.0f * cos_theta;
          } break;
          case material_type_t::e_metal: {
            math::vec3 reflected =
                math::reflect(ray.direction, normal) +
                (material.as.metal.fuzz * random.unit_vector());
            ray = bvh::ray_t::create(hit_pos + normal * 0.0001f, reflected);
            throughput *= material.as.metal.albedo;
            // terminate ray
            if (!(math::dot(ray.direction, normal) > 0)) bounce = max_bounce;
          } break;
          case material_type_t::e_dielectric: {
            bool       front_face     = math::dot(ray.direction, normal) < 0.0f;
            math::vec3 face_normal    = front_face ? normal : -normal;
            float      ri             = front_face
                                            ? (1.f / material.as.dielectric.refraction_index)
                                            : material.as.dielectric.refraction_index;
            math::vec3 unit_direction = math::normalize(ray.direction);
            float      cos_theta =
                std::min(math::dot(-unit_direction, face_normal), 1.0f);
            float sin_theta = std::sqrt(1.0f - cos_theta * cos_theta);

            bool       cannot_refract = ri * sin_theta > 1.0f;
            math::vec3 direction;
            auto       reflectance = [](float cosine, float refraction_index) {
              auto r0 = (1.0f - refraction_index) / (1.0f + refraction_index);
              r0 = r0 * r0;
              return r0 + (1.0f - r0) * std::pow((1.0f - cosine), 5);
            };
            bool will_reflect =
                cannot_refract || reflectance(cos_theta, ri) > random.randf();
            if (will_reflect) {
              direction = math::reflect(unit_direction, face_normal);
            } else {
              direction = math::refract(unit_direction, face_normal, ri);
            }
            math::vec3 offset = face_normal * 0.0001f;
            ray               = bvh::ray_t::create(
                will_reflect ? hit_pos + offset : hit_pos - offset, direction);
            throughput *= math::vec3(1.0f);
          } break;
          case material_type_t::e_emissive: {
            // terminate ray
            bounce = max_bounce;
          } break;
        }
      }
      final_color += color;
    }
    return math::vec4{final_color / static_cast<float>(max_spp), 1.f};
  });
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "render took: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << "\n";  //

  image.to_disk("test.ppm");

  return 0;
}
