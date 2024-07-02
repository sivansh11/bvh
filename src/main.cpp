#include "ecs.hpp"
#include "utils.hpp"
#include "model.hpp"
#include "bvh.hpp"
#include "bvh_traversal.hpp"
#include "ray.hpp"
#include "image.hpp"
#include "postprocessing.hpp"
#include "transform.hpp"
#include "camera.hpp"
#include "render.hpp"

#include <chrono>
#include <iostream>
#include <array>

#define time_now() std::chrono::high_resolution_clock::now()
#define ms(x) std::chrono::duration_cast<std::chrono::milliseconds>(x).count()

int main() {

    core::scene_t scene;
    {
        core::entity_id_t id = scene.create();
        {
            std::string path = "../assets/model/sponza/sponza.obj";
            core::model_t model = core::load_model_from_path(path);
            auto [aabbs, centers, triangles] = core::calculate_aabbs_centers_and_triangles_from_model(model);
            core::to_disk(triangles, path + ".triangles");
            scene.construct<std::vector<core::triangle_t>>(id) = triangles;

            core::bvh::builder_options_t builder_options {
                ._o_min_primitive_count         = 1,
                ._o_max_primitive_count         = 1,
                ._o_object_split_search_type    = core::bvh::object_split_search_type_t::e_binned_sah,
                ._o_primitive_intersection_cost = 1.1f,
                ._o_node_intersection_cost      = 1.0f,
                ._o_samples                     = 100,
            };
            core::bvh::builder_t builder{ builder_options };
            auto start_building = time_now();
            core::bvh::bvh_t bvh = builder.build(aabbs.data(), centers.data(), triangles.size());
            std::cout << "building bvh took: " << ms(time_now() - start_building) << "ms\n";
            std::cout << "blas " << builder.show_info(bvh) << '\n';
            
            auto start_post_processing = time_now();
            core::bvh::post_processing_t post_processing{ builder_options };
            // post_processing.reinsertion_optimization(bvh, 10);
            // post_processing.node_collapse_optimization(bvh);
            // scene.construct<core::bvh::flat_bvh_t>(id) = post_processing.flatten(bvh);
            // core::bvh::to_disk(post_processing.flatten(bvh), path + ".high_quality.bvh");
            scene.construct<core::bvh::flat_bvh_t>(id) = core::bvh::load(path + ".high_quality.bvh");
            std::cout << "post processing bvh took: " << ms(time_now() - start_building) << "ms\n";  
            std::cout << "blas " << builder.show_info(bvh) << '\n';
            core::transform_t& transform = scene.construct<core::transform_t>(id);
            // transform.scale = { 0.01, 0.01, 0.01 };
        }
    }
    
    core::camera_t camera{ 90.f, { 0, 2.5, 5 }, { 1, 2.5, 5 } };
    core::image_t image{ 640, 640 };

    auto per_pixel = [&]<typename blas_instance_t>(uint32_t x, uint32_t y, core::bvh::flat_bvh_t& tlas, blas_instance_t *p_blas_instances) -> core::vec4 {
        core::ray_t ray = camera.ray_gen(x, y);

        core::vec3 color = { 0, 0, 0 };
        auto hit = core::bvh::traverse(tlas, ray, p_blas_instances);
        if (hit.did_intersect()) {
            // color = core::random_color_from_hit(hit);
            color = core::heatmap(hit.node_intersection_count / 100.f);
        }

        // hit tri should be rotate

        return { color, 1 };
    };

    std::cout << "render time: " << ms(core::render(scene, camera, image, per_pixel)) << '\n';
    image.to_disk("test.ppm");

    return 0;
}