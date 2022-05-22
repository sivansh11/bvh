cmake_minimum_required(VERSION 3.10)

project(app)

file(GLOB SRC_FILES src/*.cpp)

add_subdirectory(glm)

add_executable(${PROJECT_NAME} ${SRC_FILES})

target_include_directories(${PROJECT_NAME}
    PUBLIC include
)
target_link_libraries(${PROJECT_NAME}
    glm
    pthread
)

