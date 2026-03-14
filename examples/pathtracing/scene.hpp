#ifndef SCENE_HPP
#define SCENE_HPP

#include <vector>

#include "bvh/tlas.hpp"
#include "bvh/traversal.hpp"
#include "material.hpp"

struct light_record_t {
  uint32_t instance_index;
  uint32_t triangle_index;
  float    inv_area;
};

struct scene_t {
  std::vector<tlas::blas_t>     blases;
  std::vector<tlas::instance_t> instances;
  std::vector<math::aabb_t>     instance_aabbs;
  std::vector<material_t>       materials;
  std::vector<uint32_t>         light_instance_indices;
  std::vector<light_record_t>   light_records;
  std::vector<float>            light_cdfs;
  math::vec3 (*background)(const bvh::ray_t &ray);
};

#endif
