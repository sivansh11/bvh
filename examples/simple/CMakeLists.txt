cmake_minimum_required(VERSION 3.15)
project(simple)

file(GLOB_RECURSE CPP_SRC_FILES ./*.cpp)

add_executable(simple ${CPP_SRC_FILES})

target_link_libraries(simple
	PUBLIC bvh
)

target_include_directories(simple
	PUBLIC bvh
)
