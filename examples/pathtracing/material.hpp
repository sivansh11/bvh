#ifndef MATERIAL_HPP
#define MATERIAL_HPP

#include <stdexcept>

#include "bvh/traversal.hpp"
#include "common.hpp"
#include "math/math.hpp"
#include "random.hpp"

enum class material_type_t {
  e_lambertian,
  e_metal,
  e_dielectric,
  e_light,
  e_unknown,
};

struct lambertian_t {
  math::vec3 albedo;
};

struct metal_t {
  math::vec3 albedo;
  float      fuzz;
};

struct dielectric_t {
  float refraction_index;
};

struct light_t {
  math::vec3 emission;
};

struct material_t {
  material_type_t type;
  union as_t {
    lambertian_t lambertian;
    metal_t      metal;
    dielectric_t dielectric;
    light_t      light;
  } as;
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

struct scatter_sample_t {
  math::vec3 brdf;
  float      cosine;
  float      pdf;
  bvh::ray_t outgoing_ray;
  bool       active = true;
};

inline scatter_sample_t sample(const material_t& material,
                               const bvh::ray_t& incoming_ray,
                               const math::vec3& hit_pos,
                               const math::vec3& normal, random_t& random) {
  scatter_sample_t result{};
  switch (material.type) {
    case material_type_t::e_lambertian: {
      math::vec3 scatter_direction = normal + random.unit_vector();
      if (math::length2(scatter_direction) < 1e-8f) scatter_direction = normal;
      scatter_direction = math::normalize(scatter_direction);
      float cos_theta   = math::dot(normal, scatter_direction);
      if (!cos_theta) result.active = false;
      result.brdf         = material.as.lambertian.albedo / math::pi<float>();
      result.cosine       = cos_theta;
      result.pdf          = cos_theta / math::pi<float>();
      result.outgoing_ray = bvh::ray_t::create(
          hit_pos + normal * epsilon, math::normalize(scatter_direction));
    } break;
    case material_type_t::e_metal: {
      math::vec3 reflected = math::reflect(incoming_ray.direction, normal);
      math::vec3 perturbed =
          reflected + (material.as.metal.fuzz * random.unit_vector());
      result.brdf         = material.as.metal.albedo;
      result.cosine       = 1.0f;
      result.pdf          = 1.0f;
      result.outgoing_ray = bvh::ray_t::create(hit_pos + normal * epsilon,
                                               math::normalize(perturbed));
      if (!(math::dot(result.outgoing_ray.direction, normal) > 0))
        result.active = false;
    } break;
    case material_type_t::e_dielectric: {
      bool       front_face  = math::dot(incoming_ray.direction, normal) < 0.0f;
      math::vec3 face_normal = front_face ? normal : -normal;
      float ri = front_face ? (1.f / material.as.dielectric.refraction_index)
                            : material.as.dielectric.refraction_index;
      math::vec3 unit_direction = math::normalize(incoming_ray.direction);
      float cos_theta = std::min(math::dot(-unit_direction, face_normal), 1.0f);
      float sin_theta = std::sqrt(std::max(0.0f, 1.0f - cos_theta * cos_theta));
      bool  cannot_refract = ri * sin_theta > 1.0f;
      math::vec3 direction;
      auto       reflectance = [](float cosine, float refraction_index) {
        auto r0 = (1.0f - refraction_index) / (1.0f + refraction_index);
        r0      = r0 * r0;
        return r0 + (1.0f - r0) * std::pow((1.0f - cosine), 5);
      };
      bool will_reflect =
          cannot_refract || reflectance(cos_theta, ri) > random.randf();
      if (will_reflect) {
        direction = math::reflect(unit_direction, face_normal);
      } else {
        direction = math::refract(unit_direction, face_normal, ri);
      }
      math::vec3 offset   = face_normal * epsilon;
      result.outgoing_ray = bvh::ray_t::create(
          will_reflect ? hit_pos + offset : hit_pos - offset, direction);
      result.brdf   = math::vec3(1.0f);
      result.cosine = 1.0f;
      result.pdf    = 1.0f;
    } break;
    case material_type_t::e_light: {
      result.active = false;
    } break;
    case material_type_t::e_unknown: {
      throw std::runtime_error("unknown");
    } break;
  }
  return result;
}

#endif
