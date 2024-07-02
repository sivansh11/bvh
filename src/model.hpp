#ifndef MODEL_HPP
#define MODEL_HPP

#include "algebra.hpp"
#include "aabb.hpp"
#include "common.hpp"

#include <vector>
#include <filesystem>

namespace core {

struct vertex_t {
    vec3 position{};
    vec3 normal{};
    vec2 uv{};
    vec3 tangent{};
    vec3 bi_tangent{};
};

enum texture_type_t {
    e_diffuse_map,
    e_specular_map,
    e_normal_map,
    e_diffuse_color,
};

struct texture_info_t {
    texture_type_t texture_type{};
    std::filesystem::path file_path{};
    glm::vec4 diffuse_color{};
};

struct material_description_t {
    std::vector<texture_info_t> texture_infos{};
};

struct mesh_t {
    std::vector<vertex_t> vertices{};
    std::vector<uint32_t> indices{}; 
    material_description_t material_description{};
    aabb_t aabb{};
    std::string name{};
};

struct model_t {
    std::vector<mesh_t> meshes;
};

struct model_loading_info_t {
    std::filesystem::path file_path;
    model_t model;
};

model_t load_model_from_path(const std::filesystem::path& file_path);

} // namespace core

#endif

// #ifndef MODEL_HPP
// #define MODEL_HPP

// #include "algebra.hpp"

// #define TINYOBJLOADER_IMPLEMENTATION
// #include "tiny_obj_loader.hpp"

// #include <iostream>
// #include <filesystem>
// #include <unordered_map>

// namespace core {

// template <typename type_t, typename... rest_t>
// constexpr void hash_combine(uint64_t& seed, const type_t& v, const rest_t&... rest) {
//     seed ^= std::hash<type_t>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
//     (hash_combine(seed, rest), ...);
// };

// struct vertex_t {
//     bool operator == (const vertex_t& other) const {
//         return position == other.position && normal == other.normal && uv == other.uv && color == other.color;
//     }

//     vec3 color;
//     vec3 position;
//     vec3 normal;
//     vec2 uv;
// };

// }

// template <>
// struct std::hash<core::vertex_t> {
//     size_t operator()(const core::vertex_t& vertex) const {
//         size_t seed = 0;
//         core::hash_combine(seed, static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.position.x)), 
//                                     static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.position.y)), 
//                                     static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.position.z)), 
//                                     static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.normal.x)), 
//                                     static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.normal.y)), 
//                                     static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.normal.z)), 
//                                     static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.uv.x)), 
//                                     static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.uv.y)), 
//                                     static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.color.x)), 
//                                     static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.color.y)), 
//                                     static_cast<const uint32_t>(*reinterpret_cast<const uint32_t *>(&vertex.color.z)));
//         return seed;
//     }
// };

// namespace core {

// struct mesh_t {
//     std::vector<vertex_t> vertices;
//     std::vector<uint32_t> indices;
//     std::string name;
// };

// struct model_t {
//     std::vector<mesh_t> meshes;
// };

// model_t load_model_from_obj(const std::filesystem::path& path) {

//     tinyobj::attrib_t attrib;
//     std::vector<tinyobj::shape_t> shapes;
//     std::vector<tinyobj::material_t> materials;
//     std::string warn, err;

//     if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.string().c_str())) {
//         std::cout << warn + err << '\n';
//         std::terminate();
//     }

//     model_t model;

//     std::unordered_map<vertex_t, uint32_t> unique_vertices{};
//     for (const auto& shape : shapes) {
//         mesh_t mesh;
//         mesh.name = shape.name;
//         // std::cout << shape.name << '\n';
//         for (const auto& index : shape.mesh.indices) {
//             vertex_t vertex{};

//             if (index.vertex_index >= 0) {
//                 vertex.position = vec3{
//                     static_cast<float>(attrib.vertices[3 * index.vertex_index + 0]),
//                     static_cast<float>(attrib.vertices[3 * index.vertex_index + 1]),
//                     static_cast<float>(attrib.vertices[3 * index.vertex_index + 2]),
//                 };
//                 // std::cout << attrib.vertices[3 * index.vertex_index + 0] << ' ' << attrib.vertices[3 * index.vertex_index + 1] << ' ' << attrib.vertices[3 * index.vertex_index + 2] << ' ' << '\n';
//                 vertex.color = vec3{
//                     static_cast<float>(attrib.colors[3 * index.vertex_index + 0]),
//                     static_cast<float>(attrib.colors[3 * index.vertex_index + 1]),
//                     static_cast<float>(attrib.colors[3 * index.vertex_index + 2]),
//                 };
//             }

//             if (index.normal_index >= 0) {
//                 vertex.normal = vec3{
//                     static_cast<float>(attrib.normals[3 * index.normal_index + 0]),
//                     static_cast<float>(attrib.normals[3 * index.normal_index + 1]),
//                     static_cast<float>(attrib.normals[3 * index.normal_index + 2]),
//                 };
//             }

//             if (index.texcoord_index >= 0) {
//                 vertex.uv = vec2{
//                     static_cast<float>(attrib.texcoords[2 * index.texcoord_index + 0]),
//                     static_cast<float>(attrib.texcoords[2 * index.texcoord_index + 1]),
//                 };
//             }
//             // std::cout << std::hash<vertex_t>()(vertex) << '\n';
//             // std::cout << to_string(vertex.position) << '\n';
//             if (unique_vertices.count(vertex) == 0) {
//                 unique_vertices[vertex] = static_cast<uint32_t>(mesh.vertices.size());
//                 mesh.vertices.push_back(vertex);
//             }
//             mesh.indices.push_back(unique_vertices[vertex]);
//             // mesh.indices.push_back(mesh.vertices.size());
//             // mesh.vertices.push_back(vertex);
//         }
//         model.meshes.push_back(mesh);
//     }

//     // for (auto& [vertex, index] : unique_vertices) {
//     //     std::cout << to_string(vertex.position) << ' ' << index << '\n';
//     // }
    
//     return model;
// }


// } // namespace core

// #endif