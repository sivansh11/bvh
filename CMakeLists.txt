cmake_minimum_required(VERSION 3.10)

project(bvh)

add_subdirectory(deps/glm)
set(ASSIMP_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)
add_subdirectory(deps/assimp)

file(GLOB_RECURSE SRC_FILES src/*.cpp)

add_executable(bvh ${SRC_FILES})

target_link_libraries(bvh
    glm
    assimp
)