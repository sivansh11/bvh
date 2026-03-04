#include <cassert>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "bvh/bvh.hpp"
#include "bvh/tlas.hpp"
#include "bvh/traversal.hpp"
#include "camera.hpp"
#include "glm/common.hpp"
#include "image.hpp"
#include "math/aabb.hpp"
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

struct material_t {
  math::vec3 color;
  math::vec3 emissive;
};

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: [simple] [model]\n";
    exit(EXIT_FAILURE);
  }

  std::vector<tlas::blas_t>     blases;
  std::vector<tlas::instance_t> instances;
  std::vector<math::aabb_t>     instance_aabbs;
  std::vector<material_t>       materials;

  {
    auto model =
        model::load_model_from_path("/home/sivansh/dev/plain/plain.obj");
    for (auto &mesh : model.meshes) {
      auto          triangles = model::create_triangles_from_mesh(mesh);
      auto          aabbs     = math::aabbs_from_triangles(triangles);
      tlas::blas_t &blas = blases.emplace_back(bvh::build_bvh_sweep_sah(aabbs),
                                               std::move(triangles));

    }
  }

  image_t image{1600, 1200};
  // camera_t camera{90.f, {0, 1, 2}, {0, 1, 0}};
  camera_t camera{90.f, {0, 1, 0}, {1, 1, 0}};
  camera.set_dimentions(image._width, image._height);

  auto start = std::chrono::high_resolution_clock::now();
  render(16, image, [&](uint32_t x, uint32_t y) {
    auto [O, D]    = camera.ray_gen(x, y);
    bvh::ray_t ray = bvh::ray_t::create(O, D);
    auto hit = bvh::intersect_bvh(bvh.nodes.data(), bvh.prim_indices.data(),
                                  triangles.data(), ray);
    if (hit.did_intersect()) {
      // return math::vec4{random_color_from_hit(hit.prim_index), 1};
      return turbo_color_map((((hit.node_intersections - 1) / 2.f) +
                              hit.triangle_intersections * 1.1f) /
                             150.f);
      // return turbo_color_map((hit.triangle_intersections * 1.1f) / 50.f);
      // return turbo_color_map(((hit.node_intersections - 1) / 2.f) / 150.f);
    } else {
      return math::vec4{0, 0, 0, 0};
    }
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
