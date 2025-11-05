cmake_minimum_required(VERSION 3.14)

project(uavs4smart_farming)
# Set Debug build type
set(CMAKE_BUILD_TYPE Debug)

# Enable all warnings and treat them as errors
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

add_subdirectory(MotionPlanner/ros/area_coverage)
add_subdirectory(TaskPlanner/CLI)
add_subdirectory(TaskPlanner/Qt-GUI)
add_subdirectory(TaskPlanner/Qt-GUI-paper)

