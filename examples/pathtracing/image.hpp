#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <filesystem>
#include <fstream>

#include "math/math.hpp"

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

#endif
