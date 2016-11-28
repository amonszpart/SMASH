project(SMASH)
cmake_minimum_required(VERSION 2.8)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14 -Wextra -Wall")

#LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" )

# OpenCV
SET(OpenCV_HINTS_DIR "/home/bontius/workspace/3rdparty/opencv/install/share/OpenCV")
IF (EXISTS ${OpenCV_HINTS_DIR})
    SET(OpenCV_DIR ${OpenCV_HINTS_DIR})
ENDIF()

# Eigen
FIND_PACKAGE( Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )

# VTK
find_package(VTK 6 REQUIRED vtkCommonMath vtkCommonCore vtkRenderingOpenGL vtkRenderingLOD vtkInteractionStyle)
include(${VTK_USE_FILE})
SET(COMPILE_DEFINITIONS ${COMPILE_DEFINITIONS} ${VTK_DEFINITIONS})

# Ceres
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES( ${GLOG_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} )

#FIND_PACKAGE(SuiteSparse REQUIRED)
#INCLUDE_DIRECTORIES( ${SUITESPARSE_INCLUDE_DIRS} )

include_directories(${CMAKE_SOURCE_DIR}/external)
#include_directories(${CMAKE_SOURCE_DIR}/external/trackingLinux64/include)

# common
add_subdirectory(common)
INCLUDE_DIRECTORIES( ${common_INCLUDE_DIRS} )

# visualization
add_subdirectory(vis)

# main
add_subdirectory(bundle_physics)

