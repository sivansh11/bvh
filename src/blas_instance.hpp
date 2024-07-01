#ifndef BLAS_INSTANCE_HPP
#define BLAS_INSTANCE_HPP

#include "bvh.hpp"

namespace core {

namespace bvh {

template <typename bvh_t, typename primitive_t>
struct blas_instance_t {
    static blas_instance_t create(bvh_t *p_bvh, primitive_t *p_primitive, const mat4& transform) {
        blas_instance_t blas_instance{};
        blas_instance.p_bvh = p_bvh;
        blas_instance.p_primitive = p_primitive;
        blas_instance.aabb = null_aabb;

        aabb_t root_aabb = p_bvh->root_aabb();

        for (uint32_t i = 0; i < 8; i++) {
            vec3 pos = {
                i & 1 ? root_aabb.max.x : root_aabb.min.x,
                i & 2 ? root_aabb.max.y : root_aabb.min.y,
                i & 4 ? root_aabb.max.z : root_aabb.min.z,
            };
            pos = transform * vec4(pos, 1);
            blas_instance.aabb.grow(pos);
        }
        blas_instance.inverse_transform = inverse(transform);
        return blas_instance;
    }

    bvh_t *p_bvh;
    primitive_t *p_primitive;
    aabb_t aabb;
    mat4 inverse_transform;
};

} // namespace bvh

} // namespace core

#endif