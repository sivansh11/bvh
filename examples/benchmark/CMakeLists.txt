cmake_minimum_required(VERSION 3.15)
project(benchmark)

file(GLOB_RECURSE CPP_SRC_FILES ./*.cpp)

add_executable(benchmark ${CPP_SRC_FILES})

target_link_libraries(benchmark
	PUBLIC bvh
)

target_include_directories(benchmark
	PUBLIC bvh
)
