#ifndef BVH_RAY_HPP
#define BVH_RAY_HPP

#include "math/math.hpp"

namespace bvh {

struct ray_t {
  math::vec3 origin, direction;
};

} // namespace bvh

#endif // !BVH_RAY_HPP
