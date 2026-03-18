#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <filesystem>
#include <fstream>

#include "math/math.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

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
    _p_pixels = new math::vec3[_width * _height];
  }
  ~image_t() { delete[] _p_pixels; }

  image_t(const image_t& other) : _width(other._width), _height(other._height) {
    _p_pixels = new math::vec3[_width * _height];
    std::memcpy(_p_pixels, other._p_pixels,
                _width * _height * sizeof(math::vec3));
  }

  math::vec3& at(uint32_t x, uint32_t y) {
    assert(y * _width + x < _width * _height);
    return _p_pixels[y * _width + x];
  }

  math::vec3 sample(float u, float v) const {
    u           = u - std::floor(u);
    v           = v - std::floor(v);
    float    x  = u * static_cast<float>(_width);
    float    y  = v * static_cast<float>(_height);
    uint32_t i0 = static_cast<uint32_t>(x);
    uint32_t j0 = static_cast<uint32_t>(y);
    uint32_t i1 = (i0 + 1) % _width;
    uint32_t j1 = (j0 + 1) % _height;
    float    tx = x - static_cast<float>(i0);
    float    ty = y - static_cast<float>(j0);

    math::vec3 c00 = _p_pixels[j0 * _width + i0];
    math::vec3 c10 = _p_pixels[j0 * _width + i1];
    math::vec3 c01 = _p_pixels[j1 * _width + i0];
    math::vec3 c11 = _p_pixels[j1 * _width + i1];

    math::vec3 c0 = c00 * (1.f - tx) + c10 * tx;
    math::vec3 c1 = c01 * (1.f - tx) + c11 * tx;
    return c0 * (1.f - ty) + c1 * ty;
  }

  static std::shared_ptr<image_t> load_from_path(
      const std::filesystem::path& path) {
    int width, height, channels;
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data =
        stbi_load(path.string().c_str(), &width, &height, &channels, 3);
    if (!data) {
      throw std::runtime_error("Failed to load image: " + path.string());
    }

    auto image = std::make_shared<image_t>(static_cast<uint32_t>(width),
                                           static_cast<uint32_t>(height));
    for (uint32_t i = 0; i < width * height; i++) {
      image->_p_pixels[i] = math::vec3{
          static_cast<float>(data[i * 3 + 0]) / 255.f,
          static_cast<float>(data[i * 3 + 1]) / 255.f,
          static_cast<float>(data[i * 3 + 2]) / 255.f,
      };
    }
    stbi_image_free(data);
    return image;
  }

  void to_disk(const std::filesystem::path& path) {
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
  math::vec3* _p_pixels;
};

#endif
