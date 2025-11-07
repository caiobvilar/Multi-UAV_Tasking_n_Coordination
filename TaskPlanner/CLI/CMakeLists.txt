cmake_minimum_required(VERSION 3.14)
project(BenchmarkAreaCoverage)

set(CMAKE_CXX_STANDARD 17)
include(cmake/CPM.cmake)
# include json library
CPMAddPackage(
        NAME nlohmann_json
        VERSION 3.9.1
        # not using the repo as it takes forever to clone
        URL https://github.com/nlohmann/json/releases/download/v3.9.1/include.zip
        URL_HASH SHA256=6bea5877b1541d353bd77bdfbdb2696333ae5ed8f9e8cc22df657192218cad91
)
if(nlohmann_json_ADDED)
    add_library(nlohmann_json INTERFACE)
    target_include_directories(nlohmann_json SYSTEM INTERFACE ${nlohmann_json_SOURCE_DIR}/include)
endif()


# include wykobi library for computational geometry
include_directories(include/wykobi)
# include Armadillo for matrix manipulation
find_package(Armadillo REQUIRED)
#include opencv for image loading
find_package(OpenCV REQUIRED)

include_directories(include
        /usr/include/python3.8
        ${ARMADILLO_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        )


# include boost for program_options
find_package(Boost COMPONENTS REQUIRED program_options filesystem)



add_executable(BenchmarkAreaCoverage
        main.cpp
        src/MinTurnDecomposition.cpp
        src/RotatingCalipers.cpp
        src/HorizontalAreaDecomposition.cpp
        src/TriangularDecomposition.cpp
        include/algo/TriangularDecomposition.h
        src/option_parser.cpp
        src/SweepPathCalipers.cpp
        )

target_link_libraries(${PROJECT_NAME}
        nlohmann_json
        ${ARMADILLO_LIBRARIES}
        -lpython3.8
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )