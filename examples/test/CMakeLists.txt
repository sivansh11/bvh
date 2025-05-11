cmake_minimum_required(VERSION 3.15)

project(test)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/OUTPUT/${PROJECT_NAME}")

file(GLOB_RECURSE CPP_SRC_FILES ./*.cpp)

add_executable(test ${CPP_SRC_FILES})

target_link_libraries(test
	PUBLIC bvh
)

target_include_directories(test
	PUBLIC bvh
)
