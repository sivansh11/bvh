cmake_minimum_required(VERSION 3.10)

project(bvh)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(FETCHCONTENT_QUIET FALSE)

include(FetchContent)

FetchContent_Declare(
  math
  GIT_REPOSITORY https://github.com/sivansh11/math
  GIT_TAG main
)
FetchContent_MakeAvailable(math)

FetchContent_Declare(
  model
  GIT_REPOSITORY https://github.com/sivansh11/model
  GIT_TAG main
)
FetchContent_MakeAvailable(model)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tinybvh/)
file(DOWNLOAD
    https://raw.githubusercontent.com/jbikker/tinybvh/main/tiny_bvh.h
    "${CMAKE_CURRENT_BINARY_DIR}/tinybvh/tiny_bvh.h"
    STATUS DOWNLOAD_STATUS
)
message("-- fetched tinybvh")

file(GLOB_RECURSE SRC_FILES src/*cpp)

add_library(bvh ${SRC_FILES})

target_link_libraries(bvh
  PUBLIC math
  PUBLIC model
)

target_include_directories(bvh
  PUBLIC includes
  PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/tinybvh/
)

if (BVH_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

# default built type if CMAKE_BUILD_TYPE is not set
set(DEFAULT_BUILT_TYPE "Debug")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${DEFAULT_BUILT_TYPE}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${DEFAULT_BUILT_TYPE}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()
