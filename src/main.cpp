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

#include <thread>
#include <chrono>
#include <iostream>
#include <mutex>

#define time_now() std::chrono::high_resolution_clock::now()
#define ms(x) std::chrono::duration_cast<std::chrono::milliseconds>(x).count()

core::vec3 random_color_from_hit(core::bvh::hit_t& hit) {
    return { (((hit.primitive_id + 1) * 123) % 255) / 255.f, (((hit.primitive_id + 1) * 456) % 255) / 255.f, (((hit.primitive_id + 1) * 789) % 255) / 255.f };
}

core::vec3 heatmap(float t) {
    const glm::vec3 C0 = glm::vec3(-0.020390, 0.009557, 0.018508);
    const glm::vec3 C1 = glm::vec3(3.108226, -0.106297, -1.105891);
    const glm::vec3 C2 = glm::vec3(-14.539061, -2.943057, 14.548595);
    const glm::vec3 C3 = glm::vec3(71.394557, 22.644423, -71.418400);
    const glm::vec3 C4 = glm::vec3(-152.022488, -31.024563, 152.048692);
    const glm::vec3 C5 = glm::vec3(139.593599, 12.411251, -139.604042);
    const glm::vec3 C6 = glm::vec3(-46.532952, -0.000874, 46.532928);
    return C0 + (C1 + (C2 + (C3 + (C4 + (C5 + C6 * t) * t) * t) * t) * t) * t;
}

auto render(core::scene_t& scene, core::camera_t& camera, core::image_t& image) {
    camera.set_dimentions(image.width, image.height);

    auto [tlas, blas_instances] = core::create_tlas_from_scene(scene);

    auto per_pixel = [&](uint32_t x, uint32_t y) -> core::vec4 {
        core::ray_t ray = camera.ray_gen(x, y);

        core::vec3 color = { 0, 0, 0 };
        auto hit = core::bvh::traverse(tlas, ray, blas_instances.data());
        if (hit.did_intersect()) {
            // color = random_color_from_hit(hit);
            color = heatmap(hit.node_intersection_count / 100.f);
            // color = heatmap(hit.primitive_intersection_count / 10.f);
            // std::cout << hit.primitive_intersection_count << '\n';
        }

        // hit tri should be rotate

        return { color, 1 };
    };

    #ifndef NDEBUG
    constexpr bool multi_threaded = false;
    #else 
    constexpr bool multi_threaded = true;
    #endif
    const uint32_t use_threads = 16;

    auto start = time_now();
    if constexpr (multi_threaded) {
        std::vector<uint32_t> task_input{};
        for (uint32_t y = 0; y < image.height; y++) task_input.push_back(y);
        size_t input_top = task_input.size();
        std::mutex task_input_access_mutex;
        auto thread_task = [&](uint32_t thread_id) {
            // rng_t rng{ thread_id };
            uint32_t y;
            while (input_top) {
                {
                    std::scoped_lock lock{ task_input_access_mutex };
                    if (input_top) y = task_input[--input_top];
                    else break;
                }
                [&](uint32_t y) {
                    for (uint32_t x = 0; x < image.width; x++) {
                        // image.at(x, y) = per_pixel(x, y, rng);
                        image.at(x, y) = per_pixel(x, y);
                    }
                }(y);
            }
        };

        
        std::vector<std::thread> threads;
        for (uint32_t i = 0; i < use_threads; i++) threads.emplace_back(thread_task, i);
        for (auto& thread : threads) thread.join();
    } else {
            
        // rng_t rng{ 0 };
        for (uint32_t y = 0; y < image.height; y++) for (uint32_t x = 0; x < image.width; x++) {
            // image.at(x, y) = per_pixel(x, y, rng);
            image.at(x, y) = per_pixel(x, y);
        }
    }
    auto end = time_now();
    return end - start;
}

auto get_average_render_time(core::scene_t& scene, core::camera_t& camera, core::image_t& image, uint32_t itr) {
    std::chrono::nanoseconds average{};
    for (uint32_t i = 0; i < itr; i++) {
        auto render_time = render(scene, camera, image);
        // if (i % 20 == 0) std::cout << "current average: " << ms(average / float(i)) << '\n';
        std::cout << "render took: " << ms(render_time) << '\n';
        average += render_time;
        std::stringstream s;
        s << itr - i << " iterations left";
        // std::cerr << s.str();
        // std::cerr << std::string(s.str().size(), '\b');
    }
    return average / float(itr);
}

int main() {

    core::scene_t scene;
    {
        core::entity_id_t id = scene.create();
        {
            std::string path = "../assets/model/sponza/sponza.obj";
            core::model_t model = core::load_model_from_obj(path);
            auto [aabbs, centers, triangles] = core::calculate_aabbs_centers_and_triangles_from_model(model);
            core::to_disk(triangles, path + ".triangles");
            scene.construct<std::vector<core::triangle_t>>(id) = triangles;

            core::bvh::builder_options_t builder_options {
                ._o_min_primitive_count         = 1,
                ._o_max_primitive_count         = 1000000,
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
            post_processing.reinsertion_optimization(bvh, 10);
            post_processing.node_collapse_optimization(bvh);
            scene.construct<core::bvh::flat_bvh_t>(id) = post_processing.flatten(bvh);
            // scene.construct<core::bvh::flat_bvh_t>(id) = core::bvh::load(path + ".high_quality.bvh");
            core::bvh::to_disk(post_processing.flatten(bvh), path + ".high_quality.bvh");
            std::cout << "post processing bvh took: " << ms(time_now() - start_building) << "ms\n";  
            std::cout << "blas " << builder.show_info(bvh) << '\n';
            core::transform_t& transform = scene.construct<core::transform_t>(id);
            transform.scale = { 0.01, 0.01, 0.01 };
        }
    }
    
    core::camera_t camera{ 90.f, { 0, 2.5, 0 }, { 1, 2.5, 0 } };
    core::image_t image{ 640, 640 };

    std::cout << "render time: " << ms(render(scene, camera, image)) << '\n';

    // std::cout << "#starting test\n";
    // auto average = get_average_render_time(scene, camera, image, 100);
    // std::cout << "average render time: " << ms(average) << '\n';

    image.to_disk("test.ppm");

    return 0;
}