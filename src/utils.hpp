#ifndef UTILS_HPP
#define UTILS_HPP

#include "postprocessing.hpp"
#include "transform.hpp"
#include "bvh.hpp"
#include "model.hpp"
#include "aabb.hpp"
#include "triangle.hpp"
#include "blas_instance.hpp"
#include "ecs.hpp"
#include "serilisation.hpp"

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

std::pair<bvh::flat_bvh_t, std::vector<bvh::blas_instance_t<bvh::flat_bvh_t, triangle_t>>> create_tlas_from_scene(scene_t& scene) {

    std::vector<bvh::blas_instance_t<bvh::flat_bvh_t, triangle_t>> blas_instances;
    std::vector<aabb_t> aabbs;
    std::vector<vec3> centers;

    scene.for_all<core::bvh::flat_bvh_t, std::vector<core::triangle_t>, core::transform_t>([&](core::entity_id_t id, core::bvh::flat_bvh_t& bvh, std::vector<core::triangle_t>& triangles, core::transform_t& transform) {
        bvh::blas_instance_t<bvh::flat_bvh_t, triangle_t> blas_instance = bvh::blas_instance_t<bvh::flat_bvh_t, triangle_t>::create(&bvh, triangles.data(), transform.mat4());
        blas_instances.push_back(blas_instance);
        aabbs.push_back(blas_instance.aabb);
        centers.push_back(blas_instance.aabb.center());
    });
    
    bvh::builder_options_t builder_options {
        ._o_min_primitive_count         = 1,
        ._o_max_primitive_count         = std::numeric_limits<uint32_t>::max(),
        ._o_object_split_search_type    = core::bvh::object_split_search_type_t::e_binned_sah,
        ._o_primitive_intersection_cost = 5.0f,
        ._o_node_intersection_cost      = 1.0f,
        ._o_samples                     = 100,
    };

    bvh::builder_t builder{ builder_options };
    bvh::bvh_t bvh = builder.build(aabbs.data(), centers.data(), blas_instances.size());
    // std::cout << "tlas " << builder.show_info(bvh) << '\n';
    bvh::post_processing_t post_processing{ builder_options };
    bvh::flat_bvh_t flat_bvh = post_processing.flatten(bvh);

    return { flat_bvh, blas_instances };
}

void to_disk(const std::vector<triangle_t>& triangles, const std::string& path) {
    binary_writer_t binary_writer{ path };
    uint32_t size = triangles.size();
    binary_writer.write(size);
    for (uint32_t i = 0; i < triangles.size(); i++) {
        const triangle_t& tri = triangles[i];
        binary_writer.write(tri.v0.position);
        binary_writer.write(tri.v0.normal);
        binary_writer.write(tri.v1.position);
        binary_writer.write(tri.v1.normal);
        binary_writer.write(tri.v2.position);
        binary_writer.write(tri.v2.normal);
    }
}

} // namespace core

#endif