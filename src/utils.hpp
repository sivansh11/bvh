#ifndef UTILS_HPP
#define UTILS_HPP

#include "model.hpp"
#include "aabb.hpp"
#include "triangle.hpp"

namespace core {

// this combines all the meshes into 1
auto calculate_aabbs_centers_and_triangles_from_model(const core::model_t& model) {
    struct {    
        std::vector<aabb_t>     aabbs;
        std::vector<vec3>       centers;
        std::vector<triangle_t> triangles;
    } out;

    for (auto& mesh : model.meshes) {
        for (uint32_t i = 0; i < mesh.indices.size(); i += 3) {
            triangle_t triangle {
                mesh.vertices[mesh.indices[i + 0]],
                mesh.vertices[mesh.indices[i + 1]],
                mesh.vertices[mesh.indices[i + 2]],
            };
            aabb_t aabb = null_aabb;
            aabb.grow(triangle.v0.position).grow(triangle.v1.position).grow(triangle.v2.position);
            vec3 center = (triangle.v0.position + triangle.v1.position + triangle.v2.position) / 3.f;

            out.triangles.push_back(triangle);
            out.aabbs.push_back(aabb);
            out.centers.push_back(center);
        }
    }
    return out;
}

} // namespace core

#endif