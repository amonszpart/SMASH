cmake_minimum_required(VERSION 2.8.1)
project(vis)

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED core imgproc highgui HINTS ${OpenCV_HINTS_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# VTK
find_package(VTK 6 REQUIRED vtkCommonMath vtkCommonCore vtkRenderingOpenGL vtkRenderingLOD vtkInteractionStyle)
include(${VTK_USE_FILE})
SET(COMPILE_DEFINITIONS ${COMPILE_DEFINITIONS} ${VTK_DEFINITIONS})
MESSAGE(STATUS VTK_REQUIRED_CXX_FLAGS: ${VTK_REQUIRED_CXX_FLAGS})
MESSAGE(STATUS VTK_LIBRARY_DIRS: ${VTK_LIBRARY_DIRS})

set( HEADERS
    include/tracking/vis/vis2D.h
    include/tracking/vis/visualizer.h
)

set( HPPS
    include/tracking/vis/impl/renderer.hpp
    include/tracking/vis/impl/visDebug.hpp
    include/tracking/vis/impl/visualizer.hpp
)

set( SOURCES
    src/visualizer.cpp
    src/vis2D.cpp
)

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include )

add_library(vis SHARED ${SOURCES} ${HEADERS} ${HPPS} )
link_libraries( ${PROJECT_NAME}
    ${VTK_LIBRARIES}
    ${OpenCV_LIBS}
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

set(${PROJECT_NAME}_USE_FILE
    ${VTK_USE_FILE}
    CACHE INTERNAL "${PROJECT_NAME}: Use files" FORCE)

set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include ${VTK_INCLUDE_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

set(${PROJECT_NAME}_LIBRARIES
    ${VTK_LIBRARIES}
    ${OpenCV_LIBS}
    CACHE INTERNAL "${PROJECT_NAME}: Library files" FORCE)

set(${PROJECT_NAME}_LIBRARY_DIRS
    ${VTK_LIBRARY_DIRS}
    CACHE INTERNAL "${PROJECT_NAME}: Library Directories" FORCE)
