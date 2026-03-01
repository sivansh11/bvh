cmake_minimum_required(VERSION 3.15)
project(pathtracing)

file(GLOB_RECURSE CPP_SRC_FILES ./*.cpp)

add_executable(pathtracing ${CPP_SRC_FILES})

target_link_libraries(pathtracing
	PUBLIC bvh
)

target_include_directories(pathtracing
	PUBLIC bvh
)
