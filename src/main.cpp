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
    std::string path = "../assets/model/sponza/sponza.obj";
    auto model = core::load_model_from_obj(path);

    auto [aabbs, centers, triangles] = core::calculate_aabbs_centers_and_triangles_from_model(model);

    auto builder = core::bvh::builder_t{ aabbs.data(), centers.data(), static_cast<uint32_t>(triangles.size()) };
    builder.set_triangle_intersection_cost(1.1)
           .set_node_intersection_cost(1.)
           .set_samples(100);

    auto bvh = builder.build();
    
    std::cout << builder.show_info(bvh) << '\n';

    core::bvh::post_processing_t post_processing{ bvh, builder };
    post_processing.reinsertion_optimization(100, 4);
    std::cout << builder.show_info(bvh) << '\n';
    post_processing.node_collapse_optimization();
    std::cout << builder.show_info(bvh) << '\n';
    post_processing.reinsertion_optimization(200, 1);
    std::cout << builder.show_info(bvh) << '\n';
    post_processing.node_collapse_optimization();
    std::cout << builder.show_info(bvh) << '\n';

    auto flat_bvh = post_processing.flatten();

    render(flat_bvh, triangles, "test.ppm");

    bvh::to_disk(flat_bvh, path + ".bvh");

    return 0;
}