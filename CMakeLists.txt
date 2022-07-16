cmake_minimum_required(VERSION 3.10)

project(BVH)

add_subdirectory(glm)

add_executable(example_basic_usage example/example_basic_usage.cpp)
add_executable(example_triangle_triangle_intersection example/example_triangle_triangle_intersection.cpp)
add_executable(example_bvh_instancing example/example_bvh_instancing.cpp)
add_executable(example_tlas example/example_tlas.cpp)

include_directories(BVH
    src
)

include_directories(${PROJECT_NAME}
    PUBLIC include
    glm
)
link_libraries(${PROJECT_NAME}
    glm
    pthread
)

