#ifndef RANDOM_HPP
#define RANDOM_HPP

#include <random>

#include "math/math.hpp"
#include "math/triangle.hpp"

struct random_t {
  random_t(uint32_t seed) { rng.seed(seed); }
  std::mt19937                          rng{};
  std::uniform_real_distribution<float> dist{0.0, 1.0};

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

#endif
