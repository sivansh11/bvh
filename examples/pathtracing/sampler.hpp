#ifndef SAMPLER_HPP
#define SAMPLER_HPP

#include <stdexcept>

#include "math/triangle.hpp"

enum class sampler_type_t {
  e_unknown,
  e_white_noise,
};

inline static uint32_t pcg_hash(uint32_t input) {
  uint32_t state = input * 747796405u + 2891336453u;
  uint32_t word  = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
  return (word >> 22u) ^ word;
}

struct white_noise_sampler_t {
  uint32_t state = 0;

  float randf() {
    uint32_t hash = pcg_hash(state++);
    return static_cast<float>(hash) * (1.f / 4294967296.f);
  }
};

struct sampler_t {
  sampler_type_t type = sampler_type_t::e_unknown;
  union as_t {
    white_noise_sampler_t white_noise_sampler;
  } as;
  float randf() {
    switch (type) {
      case sampler_type_t::e_white_noise:
        return as.white_noise_sampler.randf();
      default:
        throw std::runtime_error("unknown sampler");
    }
  }

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
    if (n <= 1) return 0;
    float r = randf();
    return std::min(static_cast<uint32_t>(r * n), n - 1);
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

inline sampler_t create_white_noise_sampler(uint32_t x,       //
                                            uint32_t y,       //
                                            uint32_t width,   //
                                            uint32_t height,  //
                                            uint32_t spp) {
  sampler_t sampler{};
  sampler.type = sampler_type_t::e_white_noise;
  uint32_t pixel_index = x + y * width;
  sampler.as.white_noise_sampler.state = pcg_hash(pixel_index) ^ pcg_hash(spp);
  return sampler;
}

#endif
