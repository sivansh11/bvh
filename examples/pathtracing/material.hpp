#ifndef MATERIAL_HPP
#define MATERIAL_HPP

#include <memory>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>

#include "common.hpp"
#include "glm/geometric.hpp"
#include "image.hpp"
#include "math/math.hpp"
#include "model/model.hpp"
#include "sampler.hpp"

inline bool is_normalized(const math::vec3& v) {
  return std::abs(math::length2(v) - 1.0f) < epsilon;
}

inline static std::unordered_map<std::string, std::shared_ptr<image_t>>
    texture_cache;

inline std::shared_ptr<image_t> load_texture(
    const std::filesystem::path& path) {
  std::string key = path.string();
  auto        itr = texture_cache.find(key);
  if (itr != texture_cache.end()) {
    return itr->second;
  }

  auto texture       = image_t::load_from_path(path);
  texture_cache[key] = texture;
  return texture;
}

inline std::shared_ptr<image_t> create_solid_color_image(math::vec3 color) {
  auto image      = std::make_shared<image_t>(1, 1);
  image->at(0, 0) = color;
  return image;
}

inline std::shared_ptr<image_t> get_diffuse_texture(
    const model::material_description_t& material_desc) {
  for (const auto& info : material_desc.texture_infos) {
    if (info.texture_type == model::texture_type_t::e_diffuse_map) {
      return load_texture(info.file_path);
    }
  }

  for (const auto& info : material_desc.texture_infos) {
    if (info.texture_type == model::texture_type_t::e_diffuse_color) {
      return create_solid_color_image(math::vec3{
          info.diffuse_color.x, info.diffuse_color.y, info.diffuse_color.z});
    }
  }

  assert(false && "Every mesh must have at least diffuse_color");
  return nullptr;
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
  std::shared_ptr<image_t> texture;
  bool                     is_specular() const { return false; }
  math::vec3               emitted(sampler_t&        sampler,
                                   const math::vec3& wo,  //
                                   const math::vec3& n,   //
                                   const math::vec2& uv) const {
    return math::vec3{0.f};
  }
  math::vec3 evaluate(sampler_t&        sampler,
                      const math::vec3& wi,  //
                      const math::vec3& wo,  //
                      const math::vec3& n,   //
                      const math::vec2& uv) const {
    assert(is_normalized(wi));
    assert(is_normalized(wo));
    assert(is_normalized(n));
    float cos_theta = math::dot(wi, n);
    if (cos_theta <= 0.f) return math::vec3{0.f};
    return texture->sample(uv.x, uv.y) * (1.f / math::pi<float>());
  }
  std::pair<bool, scatter_sample_t> sample(sampler_t&        sampler,
                                           const math::vec3& wo,  //
                                           const math::vec3& n,   //
                                           const math::vec2& uv) const {
    assert(is_normalized(wo));
    assert(is_normalized(n));
    scatter_sample_t scatter_sample{};
    scatter_sample.wi = n + sampler.unit_vector();
    if (math::length2(scatter_sample.wi) < 1e-8f) scatter_sample.wi = n;
    scatter_sample.wi          = math::normalize(scatter_sample.wi);
    float cos_theta            = math::dot(scatter_sample.wi, n);
    scatter_sample.pdf         = cos_theta * math::one_over_pi<float>();
    scatter_sample.attenuation = texture->sample(uv.x, uv.y);
    if (cos_theta <= 0.f) return {false, {}};
    return {true, scatter_sample};
  }
  float pdf(sampler_t&        sampler,  //
            const math::vec3& wi,       //
            const math::vec3& wo,       //
            const math::vec3& n,        //
            const math::vec2& uv) const {
    float cos_theta = math::dot(wi, n);
    return cos_theta > 0.f ? cos_theta * math::one_over_pi<float>() : 0.f;
  }
};

struct metal_t {
  math::vec3 albedo;
  float      fuzz;
  bool       is_specular() const { return true; }
  math::vec3 emitted(sampler_t&        sampler,  //
                     const math::vec3& wo,       //
                     const math::vec3& n,        //
                     const math::vec2& uv) const {
    return math::vec3{0.f};
  }
  math::vec3 evaluate(sampler_t&        sampler,  //
                      const math::vec3& wi,       //
                      const math::vec3& wo,       //
                      const math::vec3& n,        //
                      const math::vec2& uv) const {
    return math::vec3{0.f};
  }
  std::pair<bool, scatter_sample_t> sample(sampler_t&        sampler,
                                           const math::vec3& wo,  //
                                           const math::vec3& n,
                                           const math::vec2& uv) const {
    scatter_sample_t scatter_sample{};
    math::vec3       reflected = math::reflect(-wo, n);
    scatter_sample.wi =
        math::normalize(reflected + (fuzz * sampler.unit_vector()));
    scatter_sample.attenuation = albedo;
    scatter_sample.pdf         = 1.f;
    if (math::dot(scatter_sample.wi, n) <= 0) return {false, {}};
    return {true, scatter_sample};
  }
  float pdf(sampler_t&        sampler,  //
            const math::vec3& wi,       //
            const math::vec3& wo,       //
            const math::vec3& n,        //
            const math::vec2& uv) const {
    return 0.f;
  }
};

struct dielectric_t {
  float      refraction_index;
  bool       is_specular() const { return true; }
  math::vec3 emitted(sampler_t&        sampler,  //
                     const math::vec3& wo,       //
                     const math::vec3& n,        //
                     const math::vec2& uv) const {
    return math::vec3{0.f};
  }
  math::vec3 evaluate(sampler_t&        sampler,  //
                      const math::vec3& wi,       //
                      const math::vec3& wo,       //
                      const math::vec3& n,        //
                      const math::vec2& uv) const {
    return math::vec3{0.f};
  }
  std::pair<bool, scatter_sample_t> sample(sampler_t&        sampler,
                                           const math::vec3& wo,  //
                                           const math::vec3& n,
                                           const math::vec2& uv) const {
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
    if (cannot_refract || reflectance(cos_theta, ri) > sampler.randf()) {
      direction = math::reflect(-wo, outward_normal);
    } else {
      direction = math::refract(-wo, outward_normal, ri);
    }
    scatter_sample.wi = math::normalize(direction);
    return {true, scatter_sample};
  }
  float pdf(sampler_t&        sampler,  //
            const math::vec3& wi,       //
            const math::vec3& wo,       //
            const math::vec3& n,        //
            const math::vec2& uv) const {
    return 0.f;
  }
};

struct light_t {
  math::vec3 emission;
  bool       is_specular() const { return false; }
  math::vec3 emitted(sampler_t&        sampler,  //
                     const math::vec3& wo,       //
                     const math::vec3& n,        //
                     const math::vec2& uv) const {
    assert(is_normalized(wo));
    assert(is_normalized(n));
    return math::dot(wo, n) > 0.f ? emission : math::vec3{0.f};
  }
  math::vec3 evaluate(sampler_t&        sampler,  //
                      const math::vec3& wi,       //
                      const math::vec3& wo,       //
                      const math::vec3& n,        //
                      const math::vec2& uv) const {
    return math::vec3{0.f};
  }
  std::pair<bool, scatter_sample_t> sample(sampler_t&        sampler,
                                           const math::vec3& wo,  //
                                           const math::vec3& n,
                                           const math::vec2& uv) const {
    return {false, {}};
  }
  float pdf(sampler_t&        sampler,  //
            const math::vec3& wi,       //
            const math::vec3& wo,       //
            const math::vec3& n,        //
            const math::vec2& uv) const {
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

    as_t() {}
    ~as_t() {}
  } as;

 private:
  template <typename fn_t>
  auto dispatch(fn_t&& fn) const {
    switch (type) {
      case material_type_t::e_lambertian:
        return fn(as.lambertian);
      case material_type_t::e_metal:
        return fn(as.metal);
      case material_type_t::e_dielectric:
        return fn(as.dielectric);
      case material_type_t::e_light:
        return fn(as.light);

      default:
        throw std::runtime_error("unknown material");
    }
  }

  template <typename fn_t>
  auto dispatch(fn_t&& fn) {
    switch (type) {
      case material_type_t::e_lambertian:
        return fn(as.lambertian);
      case material_type_t::e_metal:
        return fn(as.metal);
      case material_type_t::e_dielectric:
        return fn(as.dielectric);
      case material_type_t::e_light:
        return fn(as.light);

      default:
        throw std::runtime_error("unknown material");
    }
  }

 public:
  material_t() : type(material_type_t::e_unknown) {}

  ~material_t() {
    if (type != material_type_t::e_unknown) {
      dispatch([](auto& m) {
        using T = std::decay_t<decltype(m)>;
        m.~T();
      });
    }
  }

  material_t(const material_t& other) : type(other.type) {
    if (type != material_type_t::e_unknown) {
      other.dispatch([this](const auto& m) {
        using T = std::decay_t<decltype(m)>;
        new (&as) T(m);
      });
    }
  }

  material_t& operator=(const material_t& other) {
    if (this == &other) return *this;
    this->~material_t();
    type = other.type;
    if (type != material_type_t::e_unknown) {
      other.dispatch([this](const auto& m) {
        using T = std::decay_t<decltype(m)>;
        new (&as) T(m);
      });
    }
    return *this;
  }

  bool is_specular() const {
    return dispatch([&](auto& m) { return m.is_specular(); });
  }
  math::vec3 emitted(sampler_t&        sampler,
                     const math::vec3& wo,  //
                     const math::vec3& n,   //
                     const math::vec2& uv) const {
    return dispatch([&](auto& m) { return m.emitted(sampler, wo, n, uv); });
  }
  math::vec3 evaluate(sampler_t&        sampler,
                      const math::vec3& wi,  //
                      const math::vec3& wo,  //
                      const math::vec3& n,   //
                      const math::vec2& uv) const {
    return dispatch(
        [&](auto& m) { return m.evaluate(sampler, wi, wo, n, uv); });
  }
  std::pair<bool, scatter_sample_t> sample(sampler_t&        sampler,
                                           const math::vec3& wo,  //
                                           const math::vec3& n,
                                           const math::vec2& uv) const {
    return dispatch([&](auto& m) { return m.sample(sampler, wo, n, uv); });
  }
  float pdf(sampler_t&        sampler,  //
            const math::vec3& wi,       //
            const math::vec3& wo,       //
            const math::vec3& n,        //
            const math::vec2& uv) const {
    return dispatch([&](auto& m) { return m.pdf(sampler, wi, wo, n, uv); });
  }
};

inline material_t create_lambertian(math::vec3 color) {
  material_t material;
  new (&material.as.lambertian) lambertian_t();
  material.type                  = material_type_t::e_lambertian;
  material.as.lambertian.texture = create_solid_color_image(color);
  return material;
}

inline material_t create_lambertian(const std::filesystem::path& texture_path) {
  material_t material;
  new (&material.as.lambertian) lambertian_t();
  material.type                  = material_type_t::e_lambertian;
  material.as.lambertian.texture = load_texture(texture_path);
  return material;
}

inline material_t create_lambertian(std::shared_ptr<image_t> texture) {
  material_t material;
  new (&material.as.lambertian) lambertian_t();
  material.type                  = material_type_t::e_lambertian;
  material.as.lambertian.texture = texture;
  return material;
}

inline material_t create_metal(math::vec3 albedo, float fuzz) {
  material_t material;
  new (&material.as.metal) metal_t();
  material.type            = material_type_t::e_metal;
  material.as.metal.albedo = albedo;
  material.as.metal.fuzz   = fuzz;
  return material;
}

inline material_t create_dielectric(float refraction_index) {
  material_t material;
  new (&material.as.dielectric) dielectric_t();
  material.type                           = material_type_t::e_dielectric;
  material.as.dielectric.refraction_index = refraction_index;
  return material;
}

inline material_t create_light(math::vec3 emission) {
  material_t material;
  new (&material.as.light) light_t();
  material.type              = material_type_t::e_light;
  material.as.light.emission = emission;
  return material;
}

#endif
