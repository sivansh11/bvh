#ifndef BVH_TRAVERSAL_HPP
#define BVH_TRAVERSAL_HPP

#include "ray.hpp"
#include "triangle.hpp"
#include "bvh.hpp"
#include "aabb.hpp"

// all intersection functions will be here

namespace core {

namespace utils {

float safe_inverse(float x) {
    static constexpr float epsilon = std::numeric_limits<float>::epsilon();
    if (glm::abs(x) <= epsilon) {
        return x >= 0 ? 1.f / epsilon : -1.f / epsilon;
    }
    return 1.f / x;
}

} // namespace utils

namespace bvh {

struct ray_data_t {
    static ray_data_t create(const ray_t& ray) {
        ray_data_t ray_data;
        ray_data.origin = ray.origin;
        ray_data.direction = ray.direction;
        ray_data.tmin = 0.0001;
        ray_data.tmax = infinity;
        ray_data.inverse_direction = { utils::safe_inverse(ray_data.direction.x), utils::safe_inverse(ray_data.direction.y), utils::safe_inverse(ray_data.direction.z) };
        return ray_data;
    }
    vec3 origin, direction;
    vec3 inverse_direction;
    float tmin, tmax;
};

struct triangle_intersection_t {
    bool did_intersect() { return is_intersect; }
    bool is_intersect;
    float t, u, v, w;
};

triangle_intersection_t triangle_intersect(const triangle_t& triangle, const ray_data_t& ray_data) {
    vec3 e1 = triangle.v0.position - triangle.v1.position;
    vec3 e2 = triangle.v2.position - triangle.v0.position;
    vec3 n = cross(e1, e2);

    vec3 c = triangle.v0.position - ray_data.origin;
    vec3 r = cross(ray_data.direction, c);
    float inverse_det = 1.0f / dot(n, ray_data.direction);

    float u = dot(r, e2) * inverse_det;
    float v = dot(r, e1) * inverse_det;
    float w = 1.0f - u - v;

    triangle_intersection_t intersection;

    if (u >= 0 && v >= 0 && w >= 0) {
        float t = dot(n, c) * inverse_det;
        if (t >= ray_data.tmin && t <= ray_data.tmax) {
            intersection.is_intersect = true;
            intersection.t = t;
            intersection.u = u;
            intersection.v = v;
            intersection.w = w;
            return intersection;
        }
    }
    intersection.is_intersect = false;
    return intersection;
}

struct aabb_intersection_t {
    bool did_intersect() { return tmin <= tmax; }
    float tmin, tmax;
};

aabb_intersection_t aabb_intersect(const aabb_t& aabb, const ray_data_t& ray_data) {
    vec3 tmin = (aabb.min - ray_data.origin) * ray_data.inverse_direction;
    vec3 tmax = (aabb.max - ray_data.origin) * ray_data.inverse_direction;

    const vec3 old_tmin = tmin;
    const vec3 old_tmax = tmax;

    tmin = min(old_tmin, old_tmax);
    tmax = max(old_tmin, old_tmax);

    float _tmin = max(tmin[0], max(tmin[1], max(tmin[2], ray_data.tmin)));
    float _tmax = min(tmax[0], min(tmax[1], min(tmax[2], ray_data.tmax)));

    aabb_intersection_t aabb_intersection = { _tmin, _tmax };
    return aabb_intersection;
}

static const uint32_t null_id = std::numeric_limits<uint32_t>::max();

struct hit_t {
    bool did_intersect() { return primitive_id != null_id; }
    uint32_t primitive_id = null_id;
    float t = infinity;  // ray hit t
    float u, v, w;
    uint32_t node_intersection_count = 0;
    uint32_t primitive_intersection_count = 0;
};

static const hit_t null_hit { .primitive_id = null_id };

template <typename T, size_t N>
struct static_stack_t {
    static_stack_t() {}

    void push(const T& val) {
        stack[top++] = val;
    }

    T pop() {
        return stack[--top];
    }

    uint32_t top = 0;
    T stack[N];
    static constexpr uint32_t size = N;
};

// TODO: better name for this
hit_t ray_traverse_bvh_triangle_intersection(const bvh_t& bvh, const ray_t& ray, triangle_t *p_triangles) {
    ray_data_t ray_data = ray_data_t::create(ray);

    hit_t hit = null_hit;

    static_stack_t<std::shared_ptr<node_t>, 32> stack;

    auto current = bvh.root;
    
    hit.node_intersection_count++;
    if (!aabb_intersect(current->aabb, ray_data).did_intersect()) return hit;

    while (true) {
        if (current->is_leaf()) {
            for (uint32_t i = 0; i < current->primitive_indices.size(); i++) {
                const uint32_t primitive_id = current->primitive_indices[i];
                hit.primitive_intersection_count++;
                triangle_intersection_t intersection = triangle_intersect(p_triangles[primitive_id], ray_data);
                if (intersection.did_intersect()) {
                    ray_data.tmax = intersection.t;
                    hit.primitive_id = primitive_id;
                    hit.t = intersection.t;
                    hit.u = intersection.u;
                    hit.v = intersection.v;
                    hit.w = intersection.w;
                }
            }
            if (stack.top == 0) return hit;
            current = stack.pop();
        } else {
            hit.node_intersection_count += 2;
            aabb_intersection_t left_intersection = aabb_intersect(current->left->aabb, ray_data);
            aabb_intersection_t right_intersection = aabb_intersect(current->right->aabb, ray_data);

            if (left_intersection.did_intersect()) {
                if (stack.top >= stack.size) return hit;
                if (right_intersection.did_intersect()) {
                    if (left_intersection.tmin <= right_intersection.tmin) {
                        stack.push(current->right);
                        current = current->left;
                    } else {
                        stack.push(current->left);
                        current = current->right;
                    }
                } else {
                    current = current->left;
                }
            } else {
                if (right_intersection.did_intersect()) {
                    current = current->right;
                } else {
                    if (stack.top == 0) return hit;
                    current = stack.pop();
                }
            }

        }
    } 

    return hit;
}

hit_t ray_traverse_bvh_triangle_intersection(const flat_bvh_t& bvh, const ray_t& ray, triangle_t *p_triangles) {
    ray_data_t ray_data = ray_data_t::create(ray);

    hit_t hit = null_hit;

    static_stack_t<uint32_t, 32> stack;

    auto current = 0;
    
    hit.node_intersection_count++;
    if (!aabb_intersect(bvh.flat_nodes[0].aabb, ray_data).did_intersect()) return hit;

    while (true) {
        const flat_node_t& flat_node = bvh.flat_nodes[current];
        if (flat_node.is_leaf()) {
            for (uint32_t i = 0; i < flat_node.primitive_count; i++) {
                const uint32_t primitive_id = bvh.primitive_indices[flat_node.first_index + i];
                hit.primitive_intersection_count++;
                triangle_intersection_t intersection = triangle_intersect(p_triangles[primitive_id], ray_data);
                if (intersection.did_intersect()) {
                    ray_data.tmax = intersection.t;
                    hit.primitive_id = primitive_id;
                    hit.t = intersection.t;
                    hit.u = intersection.u;
                    hit.v = intersection.v;
                    hit.w = intersection.w;
                }
            }
            if (stack.top == 0) return hit;
            current = stack.pop();
        } else {
            hit.node_intersection_count += 2;
            aabb_intersection_t left_intersection = aabb_intersect(bvh.flat_nodes[flat_node.first_index + 0].aabb, ray_data);
            aabb_intersection_t right_intersection = aabb_intersect(bvh.flat_nodes[flat_node.first_index + 1].aabb, ray_data);

            if (left_intersection.did_intersect()) {
                if (stack.top >= stack.size) return hit;
                if (right_intersection.did_intersect()) {
                    if (left_intersection.tmin <= right_intersection.tmin) {
                        stack.push(flat_node.first_index + 1);
                        current = flat_node.first_index + 0;
                    } else {
                        stack.push(flat_node.first_index + 0);
                        current = flat_node.first_index + 1;
                    }
                } else {
                    current = flat_node.first_index + 0;
                }
            } else {
                if (right_intersection.did_intersect()) {
                    current = flat_node.first_index + 1;
                } else {
                    if (stack.top == 0) return hit;
                    current = stack.pop();
                }
            }

        }
    } 

    return hit;
}

} // namespace bvh

} // namespace core

#endif