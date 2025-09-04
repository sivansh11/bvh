#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

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

  uint32_t _width, _height;
  math::vec4 *_p_pixels;
};

class camera_t {
public:
  camera_t(float vfov, math::vec3 from, math::vec3 at,
           math::vec3 up = {0, 1, 0})
      : _vfov(vfov), _from(from), _at(at), _up(up) {}

  void set_dimentions(uint32_t width, uint32_t height) {
    _width = width;
    _height = height;

    float aspect_ratio = float(_width) / float(_height);

    float focal_length = length(_from - _at);
    float theta = math::radians(_vfov);
    float h = tan(theta / 2.f);
    float viewport_height = 2 * h * focal_length;
    float viewport_width = viewport_height * (float(_width) / float(_height));

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

  float _vfov;
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

math::vec4 traverse(const math::vec3 O, const math::vec3 D, const math::vec3 rD,
                    const float tmax, math::vec4 *vert, uint32_t *indices,
                    bvh::gpu_node_t *nodes) {
  math::vec4 hit;
  hit.x = tmax;
  uint node = 0, stack[64], stackPtr = 0;
  while (true) {
    const bvh::gpu_node_t n = nodes[node];
    const math::vec3 lmin = n.lmin, lmax = n.lmax;
    const math::vec3 rmin = n.rmin, rmax = n.rmax;
    const uint triCount = n.triCount;
    if (triCount > 0) {
      const uint firstTri = n.firstTri;
      for (uint i = 0; i < triCount; i++) {
        const uint triIdx = indices[firstTri + i];
        const uint v0 = triIdx * 3;
        const math::vec3 vert0 = vert[v0 + 0];
        const math::vec3 vert1 = vert[v0 + 1];
        const math::vec3 vert2 = vert[v0 + 2];
        const math::vec3 edge1 = vert1 - vert0;
        const math::vec3 edge2 = vert2 - vert0;
        const math::vec3 h = math::cross(D, edge2);
        const float a = dot(edge1, h);
        if (math::abs(a) < 0.0000001f)
          continue;
        const float f = 1 / a;
        const math::vec3 s = O - vert0;
        const float u = f * math::dot(s, h);
        const math::vec3 q = math::cross(s, edge1);
        const float v = f * math::dot(D, q);
        if (u < 0 || v < 0 || u + v > 1)
          continue;
        const float d = f * dot(edge2, q);
        if (d > 0.0f && d < hit.x)
          hit = math::vec4(d, u, v, math::uintBitsToFloat(triIdx));
      }
      if (stackPtr == 0)
        break;
      node = stack[--stackPtr];
      continue;
    }
    uint left = n.left, right = n.right;
    const math::vec3 t1a = (lmin - O) * rD, t2a = (lmax - O) * rD;
    const math::vec3 t1b = (rmin - O) * rD, t2b = (rmax - O) * rD;
    const math::vec3 minta = math::min(t1a, t2a), maxta = math::max(t1a, t2a);
    const math::vec3 mintb = math::min(t1b, t2b), maxtb = math::max(t1b, t2b);
    const float tmina =
        math::max(math::max(math::max(minta.x, minta.y), minta.z), 0.f);
    const float tminb =
        math::max(math::max(math::max(mintb.x, mintb.y), mintb.z), 0.f);
    const float tmaxa =
        math::min(math::min(math::min(maxta.x, maxta.y), maxta.z), hit.x);
    const float tmaxb =
        math::min(math::min(math::min(maxtb.x, maxtb.y), maxtb.z), hit.x);
    float dist1 = tmina > tmaxa ? 1e30f : tmina;
    float dist2 = tminb > tmaxb ? 1e30f : tminb;
    if (dist1 > dist2) {
      float h = dist1;
      dist1 = dist2;
      dist2 = h;
      uint t = left;
      left = right;
      right = t;
    }
    if (dist1 == 1e30f) {
      if (stackPtr == 0)
        break;
      else
        node = stack[--stackPtr];
    } else {
      node = left;
      if (dist2 != 1e30f)
        stack[stackPtr++] = right;
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
  model = model::merge_meshes(model);
  model::raw_mesh_t &mesh = model.meshes[0];

  bvh::gpu_bvh_t bvh = bvh::build_gpu_bvh(mesh);

  image_t image{640, 420};
  camera_t camera{90.f, {0, 1, 2}, {0, 1, 0}};
  camera.set_dimentions(image._width, image._height);

  for (uint32_t y = 0; y < image._height; y++)
    for (uint32_t x = 0; x < image._width; x++) {
      auto [O, D] = camera.ray_gen(x, y);
      math::vec3 rD = {
          safe_inverse(D.x),
          safe_inverse(D.y),
          safe_inverse(D.z),
      };
      auto hit = traverse(O, D, rD, 1e30, (math::vec4 *)bvh.triangles.data(),
                          bvh.indices.data(), bvh.nodes.data());
      if (hit.x != 1e30f) {
        image.at(x, image._height - y - 1) =
            math::vec4{random_color_from_hit(math::floatBitsToUint(hit.w)), 1};
      } else {
        image.at(x, y) = math::vec4{0, 0, 0, 0};
      }
    }

  image.to_disk("test.ppm");

  return 0;
}
