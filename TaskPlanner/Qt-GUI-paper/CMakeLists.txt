cmake_minimum_required(VERSION 3.10)
project(AreaCoverage)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
# sudo apt-get install libarmadillo-dev
find_package(Armadillo REQUIRED)

# Define include directories
set(INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    ${ARMADILLO_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/gui/include
    ${PROJECT_SOURCE_DIR}/ThirdParty/qcustomplot
    ${PROJECT_SOURCE_DIR}/ThirdParty/wykobi
    ${PROJECT_SOURCE_DIR}/ThirdParty/json/include
)
include_directories(${INCLUDE_DIRS})

# Add subdirectories
add_subdirectory(gui)

# Group source files by component
set(ALGORITHM_SOURCES
    src/Alorithm/SeqAreaDecompose.cpp
    src/Alorithm/MinimumTime.cpp
    src/Alorithm/HorizontalAreaDecomposition.cpp
    src/Alorithm/RotatingCalipers.cpp
    src/Alorithm/MinTurnDecomposition.cpp
    src/Alorithm/SweepPathCalipers.cpp
)

set(ALGORITHM_HEADERS
    include/Algorithm/SeqAreaDecompose.h
    include/Algorithm/MinimumTime.h
    include/Algorithm/HorizontalAreaDecomposition.h
    include/Algorithm/RotatingCalipers.h
    include/Algorithm/MinTurnDecomposition.h
    include/Algorithm/SweepPathCalipers.h
)

set(CORE_HEADERS
    include/problem.h
    include/solver.h
    include/logger.h
    include/pch.h
)

set(CORE_SOURCES
    src/solver.cpp
)

# Define project sources
set(PROJECT_SOURCES
    ${ALGORITHM_SOURCES}
    ${ALGORITHM_HEADERS}
    ${CORE_SOURCES}
    ${CORE_HEADERS}
)

# Main executable
add_executable(${PROJECT_NAME} main.cpp ${PROJECT_SOURCES})
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    pthread
    ${ARMADILLO_LIBRARIES}
    gui
)

######################################### GOOGLE TEST ######################################################
option(BUILD_TEST "Enable Tests Feature" OFF)
if(BUILD_TEST)
    # Download and unpack googletest at configure time
    configure_file(cmake/CMakeLists.txt.in googletest-download/CMakeLists.txt)
    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download
    )
    if(result)
        message(FATAL_ERROR "CMake step for googletest failed: ${result}")
    endif()

    execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download
    )
    if(result)
        message(FATAL_ERROR "Build step for googletest failed: ${result}")
    endif()

    # Prevent overriding the parent project's compiler/linker settings on Windows
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

    # Add googletest directly to our build
    add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
        ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
        EXCLUDE_FROM_ALL
    )

    # Handle older CMake versions
    if(CMAKE_VERSION VERSION_LESS 2.8.11)
        include_directories("${gtest_SOURCE_DIR}/include")
    endif()

    # Test executable
    set(TEST_SOURCES
        src/Alorithm/HorizontalAreaDecomposition.cpp
        src/Alorithm/RotatingCalipers.cpp
        src/Alorithm/MinTurnDecomposition.cpp
        src/Alorithm/SweepPathCalipers.cpp
    )

    set(TEST_HEADERS
        include/Algorithm/HorizontalAreaDecomposition.h
        include/Algorithm/RotatingCalipers.h
        include/Algorithm/MinTurnDecomposition.h
        include/Algorithm/SweepPathCalipers.h
    )

    add_executable(test_${PROJECT_NAME}
        UnitTest/test_planning_modules.cpp
        ${TEST_SOURCES}
        ${TEST_HEADERS}
    )

    target_link_libraries(test_${PROJECT_NAME} PRIVATE gtest_main)
    add_test(NAME test_${PROJECT_NAME} COMMAND test_${PROJECT_NAME})
endif()
