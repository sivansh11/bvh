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
#include "bvh_traversal.hpp"

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

core::vec3 random_color_from_hit(core::bvh::hit_t& hit) {
    return { (((hit.primitive_id + 1) * 123) % 255) / 255.f, (((hit.primitive_id + 1) * 456) % 255) / 255.f, (((hit.primitive_id + 1) * 789) % 255) / 255.f };
}

core::vec3 heatmap(float t) {
    const core::vec3 C0 = core::vec3(-0.020390, 0.009557, 0.018508);
    const core::vec3 C1 = core::vec3(3.108226, -0.106297, -1.105891);
    const core::vec3 C2 = core::vec3(-14.539061, -2.943057, 14.548595);
    const core::vec3 C3 = core::vec3(71.394557, 22.644423, -71.418400);
    const core::vec3 C4 = core::vec3(-152.022488, -31.024563, 152.048692);
    const core::vec3 C5 = core::vec3(139.593599, 12.411251, -139.604042);
    const core::vec3 C6 = core::vec3(-46.532952, -0.000874, 46.532928);
    return C0 + (C1 + (C2 + (C3 + (C4 + (C5 + C6 * t) * t) * t) * t) * t) * t;
}

// core::vec3 heatmap(float t) {
//     const core::vec3 blue = core::vec3(0.1, 0.3, 0.8);  // Bluish
//     const core::vec3 green = core::vec3(0.4, 0.8, 0.3);  // Greenish
//     const core::vec3 red = core::vec3(0.8, 0.2, 0.1);   // Reddish
//     float factor = (t > 0.5f) ? (t - 0.5f) * 2.0f : t * 2.0f;
//     core::vec3 color = mix(blue, green, factor);
//     color = mix(color, red, factor - 1.0f);  // Blend from green to red
//     return color;
// }

// core::vec3 heatmap(float t) {
//     static const std::array<core::vec3, 5> color_map {
//         core::vec3{ 0, 0, 1 },
//         core::vec3{ 0, 0.5, 0.5 },
//         core::vec3{ 0, 1, 0 },
//         core::vec3{ 0.5, 0.5, 0 },
//         core::vec3{ 1, 0, 0 },
//     };

//     t = std::clamp(t, 0.f, 1.f);

//     uint32_t idx = static_cast<int>(t * (color_map.size() - 1));
    
//     if (idx == 0) { return color_map[0]; }
//     else if (idx == color_map.size() - 1) { return color_map.back(); }

//     float lerp_factor = std::abs(t - static_cast<float>(idx) / (color_map.size() - 1));
//     return color_map[idx] * (1.0f - lerp_factor) + color_map[idx + 1] * lerp_factor;
// }


// auto get_average_render_time(core::scene_t& scene, core::camera_t& camera, core::image_t& image, uint32_t itr) {
//     std::chrono::nanoseconds average{};
//     for (uint32_t i = 0; i < itr; i++) {
//         auto render_time = render(scene, camera, image);
//         // if (i % 20 == 0) std::cout << "current average: " << ms(average / float(i)) << '\n';
//         std::cout << "render took: " << ms(render_time) << '\n';
//         average += render_time;
//         std::stringstream s;
//         s << itr - i << " iterations left";
//         // std::cerr << s.str();
//         // std::cerr << std::string(s.str().size(), '\b');
//     }
//     return average / float(itr);
// }

} // namespace core

#endif