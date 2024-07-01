#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include "algebra.hpp"

namespace core {

struct transform_t {
    vec3 translation{ 0.f, 0.f, 0.f };
    vec3 rotation{ 0.f, 0.f, 0.f };
    vec3 scale{ 1.f, 1.f, 1.f };
    
    transform_t() = default;

    glm::mat4 mat4() const {
        glm::mat4 transform = translate(glm::mat4(1.f), translation);
        transform = rotate(transform, rotation.x, vec3(1.0f, 0.0f, 0.0f));
        transform = rotate(transform, rotation.y, vec3(0.0f, 1.0f, 0.0f));
        transform = rotate(transform, rotation.z, vec3(0.0f, 0.0f, 1.0f));
        transform = glm::scale(transform, scale);
        return transform;
    }
};

} // namespace core

#endif