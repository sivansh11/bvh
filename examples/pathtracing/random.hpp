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

#endif
