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
#include "bvh/traversal.hpp"
#include "glm/common.hpp"
#include "math/triangle.hpp"
#include "math/utilies.hpp"
#include "model/model.hpp"

float clamp(float val, float min, float max) {
  return val > max ? max : val < min ? min : val;
}

struct image_t {
  image_t(uint32_t width, uint32_t height) : _width(width), _height(height) {
    _p_pixels = new math::vec4[_width * _height];
  }
  ~image_t() { delete[] _p_pixels; }

  math::vec4 &at(uint32_t x, uint32_t y) {
    assert(y * _width + x < _width * _height);
    return _p_pixels[y * _width + x];
  }

  void to_disk(const std::filesystem::path &path) {
    std::stringstream s;
    s << "P3\n" << _width << ' ' << _height << "\n255\n";
    for (int j = _height - 1; j >= 0; j--)
      for (int i = 0; i < _width; i++) {
        math::vec4 pixel = at(i, j);
        s << uint32_t(clamp(pixel.r, 0, 1) * 255) << ' '
          << uint32_t(clamp(pixel.g, 0, 1) * 255) << ' '
          << uint32_t(clamp(pixel.b, 0, 1) * 255) << '\n';
      }

    std::ofstream file{path};
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file");
    }
    file << s.str();
    file.close();
  }

  uint32_t    _width, _height;
  math::vec4 *_p_pixels;
};

class camera_t {
 public:
  camera_t(float vfov, math::vec3 from, math::vec3 at,
           math::vec3 up = {0, 1, 0})
      : _vfov(vfov), _from(from), _at(at), _up(up) {}

  void set_dimentions(uint32_t width, uint32_t height) {
    _width  = width;
    _height = height;

    float aspect_ratio = float(_width) / float(_height);

    float focal_length    = length(_from - _at);
    float theta           = math::radians(_vfov);
    float h               = tan(theta / 2.f);
    float viewport_height = 2 * h * focal_length;
    float viewport_width  = viewport_height * (float(_width) / float(_height));

    _w = normalize(_from - _at);
    _u = normalize(cross(_up, _w));
    _v = cross(_w, _u);

    math::vec3 viewport_u = viewport_width * _u;
    math::vec3 viewport_v = viewport_height * -_v;

    _pixel_delta_u = viewport_u / float(_width);
    _pixel_delta_v = viewport_v / float(_height);

    math::vec3 viewport_upper_left =
        _from - (focal_length * _w) - viewport_u / 2.f - viewport_v / 2.f;

    _pixel_00_loc =
        viewport_upper_left + 0.5f * (_pixel_delta_u + _pixel_delta_v);
  }

  std::pair<math::vec3, math::vec3> ray_gen(uint32_t x, uint32_t y) {
    assert(_width != 0 && _height != 0);
    math::vec3 pixel_center = _pixel_00_loc + (float(x) * _pixel_delta_u) +
                              (float(y) * _pixel_delta_v);
    math::vec3 direction = pixel_center - _from;
    return {_from, direction};
  }

 private:
  uint32_t _width = 0, _height = 0;

  float      _vfov;
  math::vec3 _from;
  math::vec3 _at;
  math::vec3 _up;

  math::vec3 _pixel_00_loc;
  math::vec3 _pixel_delta_u;
  math::vec3 _pixel_delta_v;
  math::vec3 _u, _v, _w;
};

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

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: [simple] [model]\n";
    exit(EXIT_FAILURE);
  }

  model::raw_model_t model = model::load_model_from_path(argv[1]);
  model                    = model::merge_meshes(model);
  model::raw_mesh_t &mesh  = model.meshes[0];

  auto start     = std::chrono::high_resolution_clock::now();
  auto triangles = model::create_triangles_from_mesh(mesh);
  // auto aabbs     = math::aabbs_from_triangles(triangles);

  enum builder_type_t {
    e_sweep_sah,
    e_binned_sah,
    e_ploc,
  };
  bool           is_presplit    = true;
  builder_type_t builder_type   = builder_type_t::e_sweep_sah;
  bool           is_reinsertion = false;

  float    split_factor   = 0.3f;
  uint32_t num_samples    = 8;
  uint32_t min_primitives = 1;
  uint32_t max_primitives = 1;
  uint32_t grid_dim       = 1024;
  uint32_t log_bits       = 10;
  uint32_t search_radius  = 15;

  std::vector<math::aabb_t> aabbs{};
  std::vector<uint32_t>     tri_indices{};
  bvh::bvh_t                bvh{};
  if (is_presplit) {
    auto result = bvh::presplit(triangles, split_factor);
    aabbs       = result.first;
    tri_indices = result.second;
  } else {
    aabbs = math::aabbs_from_triangles(triangles);
  }
  switch (builder_type) {
    case builder_type_t::e_sweep_sah:
      bvh = bvh::build_bvh_sweep_sah(aabbs, min_primitives, max_primitives);
      break;
    case builder_type_t::e_binned_sah:
      bvh = bvh::build_bvh_binned_sah(aabbs, num_samples, min_primitives,
                                      max_primitives);
      break;
    case builder_type_t::e_ploc:
      bvh = bvh::build_bvh_ploc(aabbs, grid_dim, log_bits, search_radius);
      break;
  }
  if (is_presplit) {
    bvh::presplit_remove_indirection(bvh, tri_indices);
    bvh::presplit_remove_duplicates(bvh);
  }
  if (is_reinsertion) bvh::reinsertion_optimize(bvh, 0.001, 5);
  auto end = std::chrono::high_resolution_clock::now();

  switch (builder_type) {
    case builder_type_t::e_sweep_sah:
      std::cout << "sweep sah ";
      break;
    case builder_type_t::e_binned_sah:
      std::cout << "binned sah ";
      break;
    case builder_type_t::e_ploc:
      std::cout << "ploc ";
      break;
  }
  std::cout << "builder ";
  if (is_reinsertion) std::cout << "with reinsertions ";
  std::cout << "took: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms\n";
  switch (builder_type) {
    case builder_type_t::e_sweep_sah:
      break;
    case builder_type_t::e_binned_sah:
      std::cout << "num bins: " << num_samples << '\n';
      break;
    case builder_type_t::e_ploc:
      std::cout << "grid_dim: " << grid_dim << '\n';
      std::cout << "log_bits: " << log_bits << '\n';
      std::cout << "search_radius: " << search_radius << '\n';
      break;
  }
  if (is_presplit) {
    float total_priority = bvh::total_presplit_priority(triangles);
    std::cout << "split factor: " << split_factor << '\n';
    std::cout << "total presplit priority: " << total_priority << '\n';
    std::cout << "total presplit splits: "
              << bvh::total_presplit_splits(total_priority, split_factor,
                                            triangles)
              << '\n';
  }
  std::cout << "depth of bvh: " << bvh::depth_of_bvh(bvh) << '\n';
  float sah = bvh::sah_of_bvh(bvh);
  float epo = bvh::epo_of_bvh(bvh, triangles);
  std::cout << "sah of bvh: " << sah << '\n';
  std::cout << "epo of bvh: " << epo << '\n';
  float alpha = 0.71;
  std::cout << "sah-epo: " << ((1.f - alpha) * sah + alpha * epo) << '\n';
  std::cout << "num primitives: " << triangles.size() << '\n';
  std::cout << "num nodes: " << bvh.nodes.size() << '\n';
  uint32_t min = std::numeric_limits<uint32_t>::max();
  uint32_t max = 0;
  for (const auto &node : bvh.nodes) {
    if (!node.is_leaf()) continue;
    min = std::min(node.prim_count, min);
    max = std::max(node.prim_count, max);
  }
  std::cout << "min primitives: " << min << '\n';
  std::cout << "max primitives: " << max << '\n';

  image_t image{1600, 1200};
  // camera_t camera{90.f, {0, 1, 2}, {0, 1, 0}};
  camera_t camera{90.f, {0, 1, 0}, {1, 1, 0}};
  camera.set_dimentions(image._width, image._height);

  for (uint32_t i = 0; i < 10; i++) {
    start = std::chrono::high_resolution_clock::now();
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
    end = std::chrono::high_resolution_clock::now();
    std::cout << "render took: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                       start)
                     .count()
              << "ms" << "; "  //
              << "average time taken per ray: "
              << float(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           end - start)
                           .count()) /
                     float(image._width * image._height)
              << "ns"
              << "\n";
  }

  image.to_disk("test.ppm");

  return 0;
}
