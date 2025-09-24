cmake_minimum_required(VERSION 3.15)
project(multi_threaded)

file(GLOB_RECURSE CPP_SRC_FILES ./*.cpp)

add_executable(multi_threaded ${CPP_SRC_FILES})

target_link_libraries(multi_threaded
	PUBLIC bvh
)

target_include_directories(multi_threaded
	PUBLIC bvh
)
