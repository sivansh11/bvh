#ifndef RENDER_HPP
#define RENDER_HPP

#include "ecs.hpp"
#include "camera.hpp"
#include "image.hpp"
#include "bvh.hpp"
#include "blas_instance.hpp"
#include "utils.hpp"

#include <thread>
#include <mutex>

namespace core {

template <typename per_pixel_func_t>
auto render(core::scene_t& scene, core::camera_t& camera, core::image_t& image, per_pixel_func_t per_pixel) {
    camera.set_dimentions(image.width, image.height);

    auto [tlas, blas_instances] = core::create_tlas_from_scene(scene);

    #ifndef NDEBUG
    constexpr bool multi_threaded = false;
    #else 
    constexpr bool multi_threaded = true;
    #endif
    const uint32_t use_threads = 16;

    auto start = std::chrono::high_resolution_clock::now();
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
                        // image.at(x, y) = per_pixel(x, y, tlas, blas_instances.data(), rng);
                        image.at(x, y) = per_pixel(x, y, tlas, blas_instances.data());
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
            // image.at(x, y) = per_pixel(x, y, tlas, blas_instances.data(), rng);
            image.at(x, y) = per_pixel(x, y, tlas, blas_instances.data());
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    return end - start;
}

} // namespace core

#endif