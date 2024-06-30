#ifndef IMAGE_HPP
#define IMAGE_HPP

#include "algebra.hpp"

#include <sstream>
#include <fstream>

namespace core {

class image_t {
public:
    image_t(uint32_t width, uint32_t height) 
      : width(width), height(height) {
        _p_pixel = new vec4[width * height];
    }

    ~image_t() {
        delete[] _p_pixel;
    }

    vec4& at(uint32_t x, uint32_t y) {
        assert(y * width + x < width * height);
        return _p_pixel[y * width + x];
    }    

    void to_disk(const std::string& path) {
        std::stringstream s;
        s << "P3\n" << width << ' ' << height << "\n255\n";
        for (size_t j = 0; j < height; j++) for (size_t i = 0; i < width; i++) {
            vec4 pixel = at(i, j);
            s << uint32_t(clamp(pixel.r, 0, 1) * 255) << ' ' << uint32_t(clamp(pixel.g, 0, 1) * 255) << ' ' << uint32_t(clamp(pixel.b, 0, 1) * 255) << '\n';
        }

        std::ofstream file{ path };
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file");
        }
        file << s.str();
        file.close();
    }

    const uint32_t width, height;
private:
    float clamp(float val, float min, float max) {
        return val > max ? max : val < min ? min : val;
    }

    vec4 *_p_pixel;
};

} // namespace core

#endif