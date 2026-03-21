#ifndef SAMPLER_HPP
#define SAMPLER_HPP

#include <memory>
#include <random>

#include "image.hpp"
#include "math/math.hpp"
#include "math/triangle.hpp"

struct sampler_t {
  std::mt19937                          rng{};
  std::uniform_real_distribution<float> dist{0.0, 1.0};

  sampler_t(uint32_t x, uint32_t y, uint32_t width, uint32_t height,
            uint32_t spp) {
    rng.seed(x + (y * width) + (spp * width * height));
  }

  float randf() { return dist(rng); }

  math::vec3 unit_vector() {
    float u1 = randf();
    float u2 = randf();

    float z   = 1.0f - 2.0f * u1;
    float r   = std::sqrt(std::max(0.0f, 1.0f - z * z));
    float phi = 2.0f * math::pi<float>() * u2;

    return math::vec3{r * std::cos(phi), r * std::sin(phi), z};
  }

  math::vec3 unit_vector_on_hemisphere(const math::vec3 normal) {
    math::vec3 v = unit_vector();
    if (math::dot(v, normal) > 0.0f)
      return v;
    else
      return -v;
  }

  uint32_t index(uint32_t n) {
    std::uniform_int_distribution<uint32_t> _dist(0, n - 1);
    return _dist(rng);
  }

  math::vec3 triangle(const math::triangle_t &triangle) {
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

#endif
