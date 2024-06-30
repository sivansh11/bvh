#include <iostream>

#include "utils.hpp"
#include "model.hpp"
#include "bvh.hpp"
#include "bvh_traversal.hpp"
#include "ray.hpp"
#include "image.hpp"

#include <thread>
#include <chrono>

using namespace core;

#define time_now() std::chrono::high_resolution_clock::now()
#define ms(x) std::chrono::duration_cast<std::chrono::milliseconds>(x).count()

void render(core::bvh::bvh_t& bvh, std::vector<core::triangle_t>& triangles, std::string name) {
    core::image_t image{ 640, 640 };

    vec3 eye{  0,  2.5,  0  };
    vec3 dir{  1,  0  ,  0  };
    vec3 up {  0,  1  ,  0  };

    // vec3 eye{  0,  2.5,  5   };
    // vec3 dir{  0,  0  ,  -1  };
    // vec3 up {  0,  1  ,  0   };

    vec3 right = normalize(cross(normalize(dir), up));
    up = cross(right, normalize(dir));

    auto per_pixel = [&](uint32_t x, uint32_t y  /* TODO: add rng */) -> core::vec4 {
        float u = 2.0f * static_cast<float>(x) / static_cast<float>(image.width) - 1.0f;
        float v = 2.0f * static_cast<float>(y) / static_cast<float>(image.height) - 1.0f;

        vec3 ray_dir = dir + (u * right) + (v * up);
        ray_dir.y /= float(image.width) / float(image.height);
        ray_t ray = ray_t::create(eye, ray_dir);

        auto hit = bvh::ray_traverse_bvh_triangle_intersection(bvh, ray, triangles.data());
        if (hit.did_intersect()) {
            return core::vec4{ (((hit.primitive_id + 1) * 34567) % 255) / 255.f, (((hit.primitive_id + 1) * 9872345678) % 255) / 255.f, (((hit.primitive_id + 1) * 3498) % 255) / 255.f, 1 };
        }

        return core::vec4{ 0, 0, 0, 1 };
    };

    auto start = time_now();
    for (uint32_t y = 0; y < image.height; y++) for (uint32_t x = 0; x < image.width; x++) {
        image.at(x, y) = per_pixel(x, y);
    }
    auto end = time_now();
    std::cout << "Render took : " << ms(end - start) << "ms\n";

    image.to_disk(name);
}


void render(core::bvh::flat_bvh_t& bvh, std::vector<core::triangle_t>& triangles, std::string name) {
    core::image_t image{ 640, 640 };

    vec3 eye{  0,  2.5,  0  };
    vec3 dir{  1,  0  ,  0  };
    vec3 up {  0,  1  ,  0  };

    // vec3 eye{  0,  2.5,  5   };
    // vec3 dir{  0,  0  ,  -1  };
    // vec3 up {  0,  1  ,  0   };

    vec3 right = normalize(cross(normalize(dir), up));
    up = cross(right, normalize(dir));

    auto per_pixel = [&](uint32_t x, uint32_t y  /* TODO: add rng */) -> core::vec4 {
        float u = 2.0f * static_cast<float>(x) / static_cast<float>(image.width) - 1.0f;
        float v = 2.0f * static_cast<float>(y) / static_cast<float>(image.height) - 1.0f;

        vec3 ray_dir = dir + (u * right) + (v * up);
        ray_dir.y /= float(image.width) / float(image.height);
        ray_t ray = ray_t::create(eye, ray_dir);

        auto hit = bvh::ray_traverse_bvh_triangle_intersection(bvh, ray, triangles.data());
        if (hit.did_intersect()) {
            return core::vec4{ (((hit.primitive_id + 1) * 34567) % 255) / 255.f, (((hit.primitive_id + 1) * 9872345678) % 255) / 255.f, (((hit.primitive_id + 1) * 3498) % 255) / 255.f, 1 };
        }

        return core::vec4{ 0, 0, 0, 1 };
    };

    auto start = time_now();
    for (uint32_t y = 0; y < image.height; y++) for (uint32_t x = 0; x < image.width; x++) {
        image.at(x, y) = per_pixel(x, y);
    }
    auto end = time_now();
    std::cout << "Render took : " << ms(end - start) << "ms\n";

    image.to_disk(name);
}

int main() {
    auto model = core::load_model_from_obj("../../horizon/assets/models/sponza_bbk/SponzaMerged/SponzaMerged.obj");
    // auto model = core::load_model_from_obj("../../horizon/assets/models/corenl_box.obj");

    auto [aabbs, centers, triangles] = core::calculate_aabbs_centers_and_triangles_from_model(model);

    auto builder = core::bvh::builder_t{ aabbs.data(), centers.data(), static_cast<uint32_t>(triangles.size()) };
    builder.set_triangle_intersection_cost(1.1)
           .set_node_intersection_cost(1.)
           .set_samples(100);

    auto start_bvh_builder = time_now();
    auto bvh = builder.build();
    auto end_bvh_builder = time_now();
    std::cout << "bvh build took : " << ms(end_bvh_builder - start_bvh_builder) << "ms\n";
    
    std::cout << builder.show_info(bvh) << '\n';
    render(bvh, triangles, "actual.ppm");

    auto start_bvh_optimizer = time_now();

    core::bvh::post_process_t post_process{ bvh, builder };
    post_process.reinsertion_optimization(25, 4)
                .collapse_unnecessary_subtrees()
                .reinsertion_optimization(25, 2);

    auto flat_bvh = post_process.flatten();

    auto end_bvh_optimizer = time_now();
    std::cout << "bvh optimization took : " << ms(end_bvh_optimizer - start_bvh_optimizer) << "ms\n";

    std::cout << builder.show_info(bvh) << '\n';
    std::this_thread::sleep_for(std::chrono::seconds{ 1 });
    // render(bvh, triangles, "test.ppm");
    render(flat_bvh, triangles, "test.ppm");

    return 0;
}