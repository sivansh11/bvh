cmake_minimum_required(VERSION 3.15)
project(pathtracing)

file(GLOB_RECURSE CPP_SRC_FILES ./*.cpp)

FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb
  GIT_TAG master
)
FetchContent_MakeAvailable(stb)

add_executable(pathtracing ${CPP_SRC_FILES})
set_property(TARGET pathtracing
             PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)

target_link_libraries(pathtracing
	PUBLIC bvh
)

target_include_directories(pathtracing
	PUBLIC bvh
	PUBLIC ${stb_SOURCE_DIR}
)
