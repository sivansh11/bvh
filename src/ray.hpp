#ifndef RAY_HPP
#define RAY_HPP

#include "algebra.hpp"

namespace core {

struct ray_t {
    static ray_t create(vec3 origin, vec3 direction) {
        ray_t ray;
        ray.origin = origin;
        ray.direction = direction;
        return ray;
    }
    vec3 origin, direction;
};

} // namespace core

#endif