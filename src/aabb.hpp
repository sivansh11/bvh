#ifndef AABB_HPP
#define AABB_HPP

#include "algebra.hpp"
#include "common.hpp"

namespace core {

struct aabb_t {
    static aabb_t create(vec3 min, vec3 max) {
        aabb_t aabb;
        aabb.min = min;
        aabb.max = max;
        return aabb;
    }

    aabb_t& grow(const vec3& point) {
        min = glm::min(min, point);
        max = glm::max(max, point);
        return *this;
    }

    aabb_t& grow(const aabb_t& aabb) {
        min = glm::min(min, aabb.min);
        max = glm::max(max, aabb.max);
        return *this;
    }

    float area() const {
        const vec3 e = max - min;
        return 2 * (e.x * e.y + e.y * e.z + e.z * e.x);
    }

    vec3 min, max;
};

static const aabb_t null_aabb = aabb_t::create(vec3{ infinity }, vec3{ -infinity });

} // namespace core

#endif