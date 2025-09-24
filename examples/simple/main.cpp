#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>

#include "bvh/bvh.hpp"
#include "glm/common.hpp"
#include "math/triangle.hpp"
#include "model/model.hpp"

float safe_inverse(float x) {
  static constexpr float epsilon = std::numeric_limits<float>::epsilon();
  if (std::abs(x) <= epsilon) {
    return x >= 0 ? 1.f / epsilon : -1.f / epsilon;
  }
  return 1.f / x;
}

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

static const uint32_t null_index = std::numeric_limits<uint32_t>::max();

struct hit_t {
  bool     did_intersect() { return prim_index != null_index; }
  uint32_t prim_index = null_index;
  float    t          = 1e30;
  float    u, v;
};

struct triangle_hit_t {
  bool  did_intersect() { return _did_intersect; }
  float t, u, v;
  bool  _did_intersect;
};

struct aabb_hit_t {
  bool  did_intersect() { return tmin <= tmax; }
  float tmin, tmax;
};

struct ray_t {
  static ray_t create(math::vec3 origin, math::vec3 direction) {
    ray_t ray;
    ray.origin    = origin;
    ray.direction = direction;
    ray.inverse_direction =
        math::vec3(safe_inverse(direction.x), safe_inverse(direction.y),
                   safe_inverse(direction.z));
    ray.tmax = 1e30;
    ray.tmin = 0.0001;
    return ray;
  }
  math::vec3 origin, direction, inverse_direction;
  float      tmax, tmin;
};

triangle_hit_t intersect_triangle(const bvh::bvh_triangle_t triangle,
                                  const ray_t               ray) {
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

hit_t intersect_bvh(bvh::node_t *nodes, uint32_t node_count, uint32_t *indices,
                    uint32_t index_count, bvh::bvh_triangle_t *triangles,
                    uint32_t triangle_count, ray_t ray) {
  static const uint32_t stack_size = 16;
  uint32_t              stack[stack_size];

  hit_t hit = hit_t();

  uint32_t stack_top = 0;

  bvh::node_t root = nodes[0];
  if (!intersect_aabb(root.min, root.max, ray).did_intersect()) return hit;

  if (root.is_leaf()) {
    for (uint32_t i = 0; i < root.prim_count; i++) {
      const uint32_t            triangle_index = indices[root.index + i];
      const bvh::bvh_triangle_t triangle       = triangles[triangle_index];
      triangle_hit_t triangle_hit = intersect_triangle(triangle, ray);
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

    aabb_hit_t left_hit  = intersect_aabb(left.min, left.max, ray);
    aabb_hit_t right_hit = intersect_aabb(right.min, right.max, ray);

    uint32_t start = 0;
    uint32_t end   = 0;
    if (left_hit.did_intersect() && left.is_leaf()) {
      if (right_hit.did_intersect() && right.is_leaf()) {
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
      uint32_t                  triangle_index = indices[index];
      const bvh::bvh_triangle_t triangle       = triangles[triangle_index];
      triangle_hit_t triangle_hit = intersect_triangle(triangle, ray);
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

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: [simple] [model]\n";
    exit(EXIT_FAILURE);
  }

  model::raw_model_t model = model::load_model_from_path(argv[1]);
  model                    = model::merge_meshes(model);
  model::raw_mesh_t &mesh  = model.meshes[0];

  auto       start = std::chrono::high_resolution_clock::now();
  bvh::bvh_t bvh   = bvh::build_bvh(mesh);
  auto       end   = std::chrono::high_resolution_clock::now();
  std::cout << "builder took: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms\n";

  std::cout << "depth of bvh: " << bvh::depth_of_bvh(bvh) << '\n';
  std::cout << "cost of bvh: " << bvh::cost_of_bvh(bvh) << '\n';
  std::cout << "num nodes: " << bvh.nodes.size() << '\n';

  image_t  image{640, 420};
  camera_t camera{90.f, {0, 1, 2}, {0, 1, 0}};
  camera.set_dimentions(image._width, image._height);

  for (uint32_t y = 0; y < image._height; y++)
    for (uint32_t x = 0; x < image._width; x++) {
      auto [O, D] = camera.ray_gen(x, y);
      ray_t ray   = ray_t::create(O, D);
      auto  hit   = intersect_bvh(bvh.nodes.data(), bvh.prim_indices.data(),
                                  bvh.triangles.data(), ray);
      if (hit.did_intersect()) {
        image.at(x, image._height - y - 1) =
            math::vec4{random_color_from_hit(hit.prim_index), 1};
      } else {
        image.at(x, y) = math::vec4{0, 0, 0, 0};
      }
    }

  image.to_disk("test.ppm");

  return 0;
}
