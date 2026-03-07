#ifndef RENDER_HPP
#define RENDER_HPP

#include <iostream>
#include <thread>
#include <vector>

#include "image.hpp"
#include "random.hpp"

template <typename fn_t>
void render(uint32_t max_threads, uint32_t max_spp, uint32_t save_interval,
            image_t &image, const std::string name, fn_t fn) {
  uint32_t width  = image._width;
  uint32_t height = image._height;
  std::vector<math::vec3> accum(width * height, math::vec3{0.f});
  uint32_t                current_spp = 0;
  std::vector<std::pair<uint32_t, uint32_t>> work;
  for (uint32_t y = 0; y < height; y++)
    for (uint32_t x = 0; x < width; x++) work.emplace_back(x, y);
  while (current_spp < max_spp) {
    uint32_t batch_len = std::min(save_interval, max_spp - current_spp);
    std::vector<std::thread> threads{};
    for (uint32_t t_idx = 0; t_idx < max_threads; t_idx++) {
      threads.emplace_back([&, t_idx, batch_len]() {
        random_t thread_rng(t_idx ^ current_spp);
        for (uint32_t w_idx = t_idx; w_idx < work.size();
             w_idx += max_threads) {
          auto [x, y]        = work[w_idx];
          uint32_t pixel_idx = y * width + x;
          math::vec3 batch_sum{0.f};
          for (uint32_t s = 0; s < batch_len; s++) {
            batch_sum += fn(x, y, thread_rng);
          }
          accum[pixel_idx] += batch_sum;
        }
      });
    }
    for (auto &thread : threads) thread.join();
    current_spp += batch_len;
    float scale = 1.0f / static_cast<float>(current_spp);
    for (uint32_t i = 0; i < width * height; i++) {
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
