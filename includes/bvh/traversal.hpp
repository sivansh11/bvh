#ifndef BVH_TRAVERSAL_HPP
#define BVH_TRAVERSAL_HPP

#include "math/triangle.hpp"

namespace bvh {

struct aabb_intersection_t {
  bool did_intersect() { return tmin <= tmax; }
  float tmin, tmax;
};

} // namespace bvh

#endif // !BVH_TRAVERSAL_HPP
