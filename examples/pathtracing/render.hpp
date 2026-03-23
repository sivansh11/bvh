#ifndef RENDER_HPP
#define RENDER_HPP

#include <algorithm>
#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "image.hpp"

template <typename fn_t>
void render(uint32_t max_threads, uint32_t max_spp, uint32_t save_interval,
            image_t &image, const std::string name, fn_t fn) {
  const uint32_t          width      = image._width;
  const uint32_t          height     = image._height;
  const uint32_t          num_pixels = width * height;
  std::vector<math::vec3> accum(num_pixels, math::vec3{0.f});
  uint32_t                current_spp = 0;
  uint32_t                num_threads =
      (max_threads == 0) ? std::thread::hardware_concurrency() : max_threads;
  while (current_spp < max_spp) {
    uint32_t batch_len = std::min(save_interval, max_spp - current_spp);
    std::atomic<uint32_t>    next_row(0);
    std::vector<std::thread> threads;
    for (uint32_t t = 0; t < num_threads; ++t) {
      threads.emplace_back([&]() {
        uint32_t y;
        while ((y = next_row.fetch_add(1, std::memory_order_relaxed)) <
               height) {
          for (uint32_t x = 0; x < width; ++x) {
            uint32_t   pixel_idx = y * width + x;
            math::vec3 row_sum{0.f};
            for (uint32_t s = 0; s < batch_len; ++s) {
              row_sum += fn(x, y, current_spp + s);
            }
            accum[pixel_idx] += row_sum;
          }
        }
      });
    }
    for (auto &thread : threads) thread.join();
    current_spp += batch_len;
    float scale = 1.0f / static_cast<float>(current_spp);
    for (uint32_t i = 0; i < num_pixels; ++i) {
      uint32_t x                  = i % width;
      uint32_t y                  = i / width;
      image.at(x, height - y - 1) = math::vec4{accum[i] * scale, 1.f};
    }
    image.to_disk(name.c_str());
    std::cout << "Progress: " << current_spp << "/" << max_spp << " SPP"
              << std::endl;
  }
}

#endif
