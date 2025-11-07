
#find_package(QT REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON)
# Create code from a list of Qt designer ui files
set(CMAKE_AUTOUIC ON)

# Find the QtWidgets library
find_package(Qt5Widgets CONFIG REQUIRED)

find_package(Qt5Core REQUIRED)

find_package(Qt5PrintSupport REQUIRED)



set(CMAKE_CXX_FLAGS_COVERAGE "${CMAKE_CXX_FLAGS_RELEASE} -fprofile-arcs -ftest-coverage")

# set up a mapping so that the Release configuration for the Qt imported target is
# used in the COVERAGE CMake configuration.
set_target_properties(Qt5::Core PROPERTIES MAP_IMPORTED_CONFIG_COVERAGE "RELEASE")

#include( ${QT_USE_FILE} )
add_definitions( ${QT_DEFINITIONS} )

include_directories(include
        ../ThirdParty/qcustomplot
        ../ThirdParty/wykobi
        ${CMAKE_BINARY_DIR}
        )
set(SRC_FILES
        src/drawingboard.cpp
        mainwindow.cpp
        mainwindow.ui
        ../ThirdParty/qcustomplot/qcustomplot.cpp
        )
set(HEADER_FILES
        include/drawingboard.h
        include/mainwindow.h
        ../ThirdParty/qcustomplot/qcustomplot.h

        )

add_library(gui
        ${SRC_FILES}
        ${HEADER_FILES}
        )

target_link_libraries(gui
        Qt5::Widgets
        Qt5::PrintSupport
        Qt5::Core
        ${catkin_LIBRARIES})
