#ifndef CONFIG_YAML
#define CONFIG_YAML

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "camera.hpp"
#include "material.hpp"
#include "math/math.hpp"

struct entity_t {
  std::string           name;
  std::filesystem::path path;
  math::mat4            transform;
  material_t            material;
};

struct config_t {
  bool                  nee;
  bool                  mis;
  uint32_t              max_spp;
  camera_t              camera;
  math::vec3            background;
  std::vector<entity_t> entities;
};

namespace yaml_utils {
inline const YAML::Node require(const YAML::Node&  node,
                                const std::string& key) {
  if (!node[key]) {
    throw std::runtime_error("Missing required YAML key: '" + key + "'");
  }
  return node[key];
}

inline math::vec3 parse_vec3(const YAML::Node& node) {
  if (!node.IsSequence() || node.size() != 3) {
    throw std::runtime_error("Expected a sequence of 3 floats for vec3");
  }
  return math::vec3{node[0].as<float>(), node[1].as<float>(),
                    node[2].as<float>()};
}

inline math::vec3 parse_vec3_opt(const YAML::Node& node, const std::string& key,
                                 const math::vec3& fallback = {0, 0, 0}) {
  if (node[key]) {
    return parse_vec3(node[key]);
  }
  return fallback;
}
}  // namespace yaml_utils

inline config_t load_config(const std::filesystem::path& path) {
  config_t   config{};
  YAML::Node yaml;

  try {
    yaml = YAML::LoadFile(path.string());
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Failed to parse YAML file: " +
                             std::string(e.what()));
  }

  auto settings  = yaml_utils::require(yaml, "settings");
  config.nee     = yaml_utils::require(settings, "nee").as<bool>();
  config.mis     = yaml_utils::require(settings, "mis").as<bool>();
  config.max_spp = yaml_utils::require(settings, "spp").as<uint32_t>();
  config.background =
      yaml_utils::parse_vec3(yaml_utils::require(settings, "background"));

  auto camera = yaml_utils::require(yaml, "camera");
  config.camera =
      camera_t{yaml_utils::require(camera, "fov").as<float>(),
               yaml_utils::parse_vec3(yaml_utils::require(camera, "from")),
               yaml_utils::parse_vec3(yaml_utils::require(camera, "to"))};

  auto entities = yaml_utils::require(yaml, "entities");
  if (!entities.IsSequence()) {
    throw std::runtime_error("'entities' must be a YAML sequence");
  }

  for (const auto& entry : entities) {
    entity_t entity{};

    // TODO: error out if path not given, unnamed_entity default is fine
    entity.name = entry["name"].as<std::string>("unnamed_entity");
    entity.path = entry["path"].as<std::string>("");

    math::vec3 position{0.f};
    math::vec3 scale{1.f};
    math::vec3 rotation{0.f};

    if (auto t = entry["transform"]) {
      position = yaml_utils::parse_vec3_opt(t, "position", position);
      scale    = yaml_utils::parse_vec3_opt(t, "scale", scale);
      rotation = yaml_utils::parse_vec3_opt(t, "rotation", rotation);
    }

    entity.transform = math::translate(math::mat4{1.f}, position) *
                       math::toMat4(math::quat{rotation}) *
                       math::scale(math::mat4{1.f}, scale);

    auto        m    = yaml_utils::require(entry, "material");
    std::string type = yaml_utils::require(m, "type").as<std::string>();

    if (type == "light") {
      math::vec3 emission =
          yaml_utils::parse_vec3(yaml_utils::require(m, "emission"));
      entity.material = create_light(emission);
    } else {
      entity.material = create_lambertian(math::vec3{1.f, 1.f, 1.f});
    }

    config.entities.push_back(entity);
  }

  return config;
}

#endif
