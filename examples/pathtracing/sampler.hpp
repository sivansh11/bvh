#ifndef SAMPLER_HPP
#define SAMPLER_HPP

#include <limits>
#include <stdexcept>

#include "math/triangle.hpp"

enum class sampler_type_t {
  e_unknown,
  e_white_noise,
  e_golden_ratio,
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

/*
 * golden ratio sampler needs to be random initially (blue ?), but follow golden
 * ration in spp
 *
 * dimension 0 spp 0 -> random
 * dimension 1 spp 0 -> random
 *
 * dimension 0 spp n -> random + n * phi
 * dimension 1 spp n -> random + n * phi
 *
 */
struct golden_ratio_sampler_t {
  uint32_t    x;
  uint32_t    width;
  uint32_t    y;
  uint32_t    height;
  uint32_t    spp;
  uint32_t    dimension;
  const float phi = 0.6180339887498948482f;
  // every randf call increases the dimension
  // for now initial offset is white
  float randf() {
    // TODO: sample blue noise for initial offset
    uint32_t pixel_index = x + (y * width);
    uint32_t seed        = pcg_hash(pixel_index) ^ pcg_hash(dimension++);
    float    offset      = static_cast<float>(seed) * (1.f / 4294967296.f);
    float    v           = offset + spp * phi;
    return v - std::floor(v);
  }
};

struct sampler_t {
  sampler_type_t type = sampler_type_t::e_unknown;
  union as_t {
    white_noise_sampler_t  white_noise_sampler;
    golden_ratio_sampler_t golden_ratio_sampler;
  } as;
  float randf() {
    switch (type) {
      case sampler_type_t::e_white_noise:
        return as.white_noise_sampler.randf();
      case sampler_type_t::e_golden_ratio:
        return as.golden_ratio_sampler.randf();
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
  sampler.type                         = sampler_type_t::e_white_noise;
  uint32_t pixel_index                 = x + y * width;
  sampler.as.white_noise_sampler.state = pcg_hash(pixel_index) ^ pcg_hash(spp);
  return sampler;
}

inline sampler_t create_golden_ratio_sampler(uint32_t x,       //
                                             uint32_t y,       //
                                             uint32_t width,   //
                                             uint32_t height,  //
                                             uint32_t spp) {
  sampler_t sampler{};
  sampler.type                              = sampler_type_t::e_golden_ratio;
  sampler.as.golden_ratio_sampler.dimension = 0;
  sampler.as.golden_ratio_sampler.x         = x;
  sampler.as.golden_ratio_sampler.y         = y;
  sampler.as.golden_ratio_sampler.width     = width;
  sampler.as.golden_ratio_sampler.height    = height;
  sampler.as.golden_ratio_sampler.spp       = spp;
  return sampler;
}

#endif
