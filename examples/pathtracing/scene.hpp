#ifndef SCENE_HPP
#define SCENE_HPP

#include <functional>
#include <vector>

#include "bvh/tlas.hpp"
#include "bvh/traversal.hpp"
#include "material.hpp"
#include "model/model.hpp"

struct light_record_t {
  uint32_t instance_index;
  uint32_t triangle_index;
  float    inv_area;
  float    probability;
};

struct instance_light_map_t {
  std::vector<uint32_t> triangle_to_light_record;
};

struct scene_t {
  std::vector<tlas::blas_t>                        blases;
  std::vector<tlas::instance_t>                    instances;
  std::vector<math::aabb_t>                        instance_aabbs;
  std::vector<material_t>                          materials;
  std::vector<uint32_t>                            light_instance_indices;
  std::vector<light_record_t>                      light_records;
  std::vector<float>                               light_cdfs;
  std::vector<instance_light_map_t>                instance_light_maps;
  std::vector<model::raw_mesh_t>                   raw_meshes;
  std::vector<uint32_t>                            instance_to_mesh_index;
  std::function<math::vec3(const bvh::ray_t& ray)> background;
};

#endif
