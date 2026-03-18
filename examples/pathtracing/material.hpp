#ifndef MATERIAL_HPP
#define MATERIAL_HPP

#include <stdexcept>

#include "bvh/traversal.hpp"
#include "common.hpp"
#include "glm/geometric.hpp"
#include "math/math.hpp"
#include "random.hpp"

inline bool is_normalized(const math::vec3& v) {
  return std::abs(math::length2(v) - 1.0f) < epsilon;
}

enum class material_type_t {
  e_unknown,
  e_lambertian,
  e_metal,
  e_dielectric,
  e_light,
};

struct scatter_sample_t {
  math::vec3 wi;
  math::vec3 attenuation;
  float      pdf;
};

struct lambertian_t {
  math::vec3 albedo;
  bool       is_specular() const { return false; }
  math::vec3 emitted(random_t&         random,
                     const math::vec3& wo,  //
                     const math::vec3& n) const {
    return math::vec3{0.f};
  }
  math::vec3 evaluate(random_t&         random,
                      const math::vec3& wi,  //
                      const math::vec3& wo,  //
                      const math::vec3& n) const {
    assert(is_normalized(wi));
    assert(is_normalized(wo));
    assert(is_normalized(n));
    float cos_theta = math::dot(wi, n);
    if (cos_theta <= 0.f) return math::vec3{0.f};
    return albedo * (1.f / math::pi<float>());
  }
  std::pair<bool, scatter_sample_t> sample(random_t&         random,
                                           const math::vec3& wo,  //
                                           const math::vec3& n) const {
    assert(is_normalized(wo));
    assert(is_normalized(n));
    scatter_sample_t scatter_sample{};
    scatter_sample.wi = n + random.unit_vector();
    if (math::length2(scatter_sample.wi) < 1e-8f) scatter_sample.wi = n;
    scatter_sample.wi          = math::normalize(scatter_sample.wi);
    float cos_theta            = math::dot(scatter_sample.wi, n);
    scatter_sample.pdf         = cos_theta * math::one_over_pi<float>();
    scatter_sample.attenuation = albedo;
    if (cos_theta <= 0.f) return {false, {}};
    return {true, scatter_sample};
  }
  float pdf(random_t&         random,  //
            const math::vec3& wi,      //
            const math::vec3& wo,      //
            const math::vec3& n) const {
    float cos_theta = math::dot(wi, n);
    return cos_theta > 0.f ? cos_theta * math::one_over_pi<float>() : 0.f;
  }
};

struct metal_t {
  math::vec3 albedo;
  float      fuzz;
  bool       is_specular() const { return true; }
  math::vec3 emitted(random_t&         random,  //
                     const math::vec3& wo,      //
                     const math::vec3& n) const {
    return math::vec3{0.f};
  }
  math::vec3 evaluate(random_t&         random,  //
                      const math::vec3& wi,      //
                      const math::vec3& wo,      //
                      const math::vec3& n) const {
    return math::vec3{0.f};
  }
  std::pair<bool, scatter_sample_t> sample(random_t&         random,
                                           const math::vec3& wo,  //
                                           const math::vec3& n) const {
    scatter_sample_t scatter_sample{};
    math::vec3       reflected = math::reflect(-wo, n);
    scatter_sample.wi =
        math::normalize(reflected + (fuzz * random.unit_vector()));
    scatter_sample.attenuation = albedo;
    scatter_sample.pdf         = 1.f;
    if (math::dot(scatter_sample.wi, n) <= 0) return {false, {}};
    return {true, scatter_sample};
  }
  float pdf(random_t&         random,  //
            const math::vec3& wi,      //
            const math::vec3& wo,      //
            const math::vec3& n) const {
    return 0.f;
  }
};

struct dielectric_t {
  float      refraction_index;
  bool       is_specular() const { return true; }
  math::vec3 emitted(random_t&         random,  //
                     const math::vec3& wo,      //
                     const math::vec3& n) const {
    return math::vec3{0.f};
  }
  math::vec3 evaluate(random_t&         random,  //
                      const math::vec3& wi,      //
                      const math::vec3& wo,      //
                      const math::vec3& n) const {
    return math::vec3{0.f};
  }
  std::pair<bool, scatter_sample_t> sample(random_t&         random,
                                           const math::vec3& wo,  //
                                           const math::vec3& n) const {
    scatter_sample_t scatter_sample{};
    scatter_sample.pdf         = 1.f;
    scatter_sample.attenuation = math::vec3{1.f};
    bool       front_face      = math::dot(wo, n) > 0;
    float      ri = front_face ? (1.f / refraction_index) : refraction_index;
    math::vec3 outward_normal = front_face ? n : -n;
    float      cos_theta      = std::min(math::dot(wo, outward_normal), 1.f);
    float      sin_theta      = std::sqrt(1.f - cos_theta * cos_theta);
    bool       cannot_refract = ri * sin_theta > 1.f;
    auto       reflectance    = [](float cosine, float refraction_index) {
      auto r0 = (1.0f - refraction_index) / (1.0f + refraction_index);
      r0      = r0 * r0;
      return r0 + (1.0f - r0) * std::pow((1.0f - cosine), 5);
    };
    math::vec3 direction;
    if (cannot_refract || reflectance(cos_theta, ri) > random.randf()) {
      direction = math::reflect(-wo, outward_normal);
    } else {
      direction = math::refract(-wo, outward_normal, ri);
    }
    scatter_sample.wi = math::normalize(direction);
    return {true, scatter_sample};
  }
  float pdf(random_t&         random,  //
            const math::vec3& wi,      //
            const math::vec3& wo,      //
            const math::vec3& n) const {
    return 0.f;
  }
};

struct light_t {
  math::vec3 emission;
  bool       is_specular() const { return false; }
  math::vec3 emitted(random_t&         random,  //
                     const math::vec3& wo,      //
                     const math::vec3& n) const {
    assert(is_normalized(wo));
    assert(is_normalized(n));
    return math::dot(wo, n) > 0.f ? emission : math::vec3{0.f};
  }
  math::vec3 evaluate(random_t&         random,  //
                      const math::vec3& wi,      //
                      const math::vec3& wo,      //
                      const math::vec3& n) const {
    return math::vec3{0.f};
  }
  std::pair<bool, scatter_sample_t> sample(random_t&         random,
                                           const math::vec3& wo,  //
                                           const math::vec3& n) const {
    return {false, {}};
  }
  float pdf(random_t&         random,  //
            const math::vec3& wi,      //
            const math::vec3& wo,      //
            const math::vec3& n) const {
    return 0.f;
  }
};

struct material_t {
  material_type_t type;
  union as_t {
    lambertian_t lambertian;
    metal_t      metal;
    dielectric_t dielectric;
    light_t      light;
  } as;
  bool is_specular() const {
    switch (type) {
      case material_type_t::e_lambertian:
        return as.lambertian.is_specular();
        break;
      case material_type_t::e_metal:
        return as.metal.is_specular();
        break;
      case material_type_t::e_dielectric:
        return as.dielectric.is_specular();
        break;
      case material_type_t::e_light:
        return as.light.is_specular();
        break;
      default:
        throw std::runtime_error("unknown material");
        break;
    }
  }
  math::vec3 emitted(random_t&         random,  //
                     const math::vec3& wo,      //
                     const math::vec3& n) const {
    switch (type) {
      case material_type_t::e_lambertian:
        return as.lambertian.emitted(random, wo, n);
        break;
      case material_type_t::e_metal:
        return as.metal.emitted(random, wo, n);
        break;
      case material_type_t::e_dielectric:
        return as.dielectric.emitted(random, wo, n);
        break;
      case material_type_t::e_light:
        return as.light.emitted(random, wo, n);
        break;
      default:
        throw std::runtime_error("unknown material");
        break;
    }
  }
  math::vec3 evaluate(random_t&         random,  //
                      const math::vec3& wi,      //
                      const math::vec3& wo,      //
                      const math::vec3& n) const {
    switch (type) {
      case material_type_t::e_lambertian:
        return as.lambertian.evaluate(random, wi, wo, n);
        break;
      case material_type_t::e_metal:
        return as.metal.evaluate(random, wi, wo, n);
        break;
      case material_type_t::e_dielectric:
        return as.dielectric.evaluate(random, wi, wo, n);
        break;
      case material_type_t::e_light:
        return as.light.evaluate(random, wi, wo, n);
        break;
      default:
        throw std::runtime_error("unknown material");
        break;
    }
  }
  std::pair<bool, scatter_sample_t> sample(random_t&         random,
                                           const math::vec3& wo,  //
                                           const math::vec3& n) const {
    switch (type) {
      case material_type_t::e_lambertian:
        return as.lambertian.sample(random, wo, n);
        break;
      case material_type_t::e_metal:
        return as.metal.sample(random, wo, n);
        break;
      case material_type_t::e_dielectric:
        return as.dielectric.sample(random, wo, n);
        break;
      case material_type_t::e_light:
        return as.light.sample(random, wo, n);
        break;
      default:
        throw std::runtime_error("unknown material");
        break;
    }
  }
  float pdf(random_t&         random,  //
            const math::vec3& wi,      //
            const math::vec3& wo,      //
            const math::vec3& n) const {
    switch (type) {
      case material_type_t::e_lambertian:
        return as.lambertian.pdf(random, wi, wo, n);
        break;
      case material_type_t::e_metal:
        return as.metal.pdf(random, wi, wo, n);
        break;
      case material_type_t::e_dielectric:
        return as.dielectric.pdf(random, wi, wo, n);
        break;
      case material_type_t::e_light:
        return as.light.pdf(random, wi, wo, n);
        break;
      default:
        throw std::runtime_error("unknown material");
        break;
    }
  }
};

inline material_t create_lambertian(math::vec3 albedo) {
  material_t material;
  material.type                 = material_type_t::e_lambertian;
  material.as.lambertian.albedo = albedo;
  return material;
}

inline material_t create_metal(math::vec3 albedo, float fuzz) {
  material_t material;
  material.type            = material_type_t::e_metal;
  material.as.metal.albedo = albedo;
  material.as.metal.fuzz   = fuzz;
  return material;
}

inline material_t create_dielectric(float refraction_index) {
  material_t material;
  material.type                           = material_type_t::e_dielectric;
  material.as.dielectric.refraction_index = refraction_index;
  return material;
}

inline material_t create_light(math::vec3 emission) {
  material_t material;
  material.type              = material_type_t::e_light;
  material.as.light.emission = emission;
  return material;
}

#endif
