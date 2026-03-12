#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <filesystem>
#include <fstream>

#include "math/math.hpp"

float clamp(float val, float min, float max) {
  return val > max ? max : val < min ? min : val;
}

inline math::vec3 exposure(math::vec3 p, float exposure) {
  float r = p.r * exposure;
  float g = p.g * exposure;
  float b = p.b * exposure;
  return {r, g, b};
}

inline math::vec3 reinhard_tonemap(math::vec3 p) {
  float r = p.r / (p.r + 1.0f);
  float g = p.g / (p.g + 1.0f);
  float b = p.b / (p.b + 1.0f);
  return {r, g, b};
}

inline math::vec3 gamma_correction(math::vec3 p, float inv_gamma) {
  float r = math::pow(p.r, inv_gamma);
  float g = math::pow(p.g, inv_gamma);
  float b = math::pow(p.b, inv_gamma);
  return {r, g, b};
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
        math::vec3 pixel = at(i, j);

        pixel = exposure(pixel, 1.f);

        pixel = reinhard_tonemap(pixel);

        pixel = gamma_correction(pixel, 1.f / 2.2f);

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

#endif
