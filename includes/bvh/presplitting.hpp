#ifndef BVH_PRE_SPLITTING_HPP
#define BVH_PRE_SPLITTING_HPP

#include <utility>
#include <vector>

#include "math/aabb.hpp"

#include "bvh/bvh.hpp"

namespace bvh {

std::pair<std::vector<math::aabb_t>, std::vector<uint32_t>> presplit(const std::bvh_tri);

}

#endif
