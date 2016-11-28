cmake_minimum_required(VERSION 2.8.1)

project(common CXX)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++14")

# OpenCV
MESSAGE(STATUS OpenCV_HINTS_DIR ${OpenCV_HINTS_DIR})
FIND_PACKAGE(OpenCV REQUIRED viz imgproc highgui core
    HINTS ${OpenCV_HINTS_DIR})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

set(HEADERS
    ../external/picojson/picojson.h
    ../external/picojson/picojsonUtil.h
    include/tracking/common/assignments.h
    include/tracking/common/correspondence.h
    include/tracking/common/defines.h
    include/tracking/common/debugEnums.h
    include/tracking/common/eigen.h
    include/tracking/common/mapper.h
    include/tracking/common/optParams.h
    include/tracking/common/trackPoint3d.h
    include/tracking/common/trackPointFwDecl.h
    include/tracking/common/track.h
    include/tracking/common/trackFwDecl.h
    include/tracking/common/tracks.h
    include/tracking/common/tracksFwDecl.h
    include/tracking/common/groupedTracks.h
    include/tracking/common/groupedTracksFwDecl.h
    #include/tracking/common/io/broxIO.h
    include/tracking/common/io/sequentialReader.h
    include/tracking/common/io/optionsParser.h
    include/tracking/common/io/segmentedTracksIo.h
    include/tracking/common/io/rgbIo.h
    include/tracking/common/io/depthIo.h
    include/tracking/common/io/pgm.h
    include/tracking/common/io/os.h
    include/tracking/common/io/savePly.h
    include/tracking/common/io/ceresOptionsIo.h
    include/tracking/common/io/assignmentsIo.h
    include/tracking/common/maths/sphericalParametrization.h
    include/tracking/common/proc/tracksByLength.h
    include/tracking/common/proc/neighbours.h
    include/tracking/common/segmentation/segmentationProblem.h
    include/tracking/common/util/util.h
    include/tracking/common/util/exception.h
    include/tracking/common/util/ceresEulerYXY.h
    include/tracking/common/util/gnuPlotter.h
    include/tracking/common/util/energyPlotter.h
    include/tracking/common/util/parse.h
    include/tracking/common/util/colorGradient.h
    include/tracking/common/maths/geometry.h
    include/tracking/common/maths/intersections.h
    include/tracking/common/maths/quaternion.h
    include/tracking/common/trackPoint2d.h
    include/tracking/common/util/eigenUtil.h
    include/tracking/common/io/cloudIo.h
    include/tracking/common/util/prettyPrint.h
    include/tracking/common/io/flowIo.h
    include/tracking/common/macros.inl
    include/tracking/common/io/matrixIo.h
    include/tracking/common/clouds/kdCloud.h
    include/tracking/common/clouds/kdCloudFwDecl.h
    include/am/stlContainers.hpp
    include/ceres/myAutoDiffCostFunction.h
    include/ceres/typedefs.h
    include/ceres/functorInfo.h)

set(HPPS
    include/ceres/impl/functorInfo.hpp
    include/tracking/common/impl/track.hpp
    include/tracking/common/impl/tracks.hpp
    include/tracking/common/impl/groupedTracks.hpp
    include/tracking/common/impl/assignments.hpp
    #include/tracking/common/io/impl/broxIO.hpp
    include/tracking/common/io/impl/rgbIo.hpp
    include/tracking/common/io/impl/segmentedTracksIo.hpp
    include/tracking/common/proc/impl/tracksByLength.hpp
    include/tracking/common/util/impl/ceresEulerYXY.hpp
    include/tracking/common/maths/impl/geometry.hpp
    include/tracking/common/maths/impl/quaternion.hpp
    include/tracking/common/maths/impl/intersections.hpp
    include/tracking/common/proc/impl/neighbours.hpp
    include/tracking/common/util/orderedPair.hpp
    include/tracking/common/impl/trackPoint3d.hpp
    include/tracking/common/io/impl/assignmentsIo.hpp
    include/tracking/common/io/impl/savePly.hpp
    include/tracking/common/io/impl/matrixIo.hpp
    include/tracking/common/clouds/impl/kdCloud.hpp
    include/tracking/common/clouds/impl/cloudFactory.hpp)

set(SOURCES
    src/assignments.cpp
    src/correspondence.cpp
    src/groupedTracks.cpp
    src/mapper.cpp
    src/track.cpp
    src/tracks.cpp
    src/trackPoint3d.cpp
    src/ceres/functorInfo.cpp
    #src/io/broxIO.cpp
    src/io/sequentialReader.cpp
    src/io/optionsParser.cpp
    src/io/rgbIo.cpp
    #src/io/convert.cpp
    src/io/segmentedTracksIo.cpp
    src/io/pgm.cpp
    src/io/depthIo.cpp
    src/io/savePly.cpp
    src/io/cloudIo.cpp
    src/io/flowIo/flowIO.cpp
    src/maths/geometry.cpp
    src/pointCloudEigen.cpp
    src/proc/neighbours.cpp
    src/proc/tracksByLength.cpp
    src/util/util.cpp
    src/io/matrixIo.cpp
    src/clouds/kdCloud.cpp
    src/clouds/cloudFactory.cpp)

include_directories( ${PROJECT_SOURCE_DIR}/include )
add_library(common SHARED ${SOURCES} ${HEADERS} ${HPPS} )
message(STATUS OpenCV_LIBS: ${OpenCV_LIBS})
target_link_libraries(common ${OpenCV_LIBS})

set_target_properties(common PROPERTIES POSITION_INDEPENDENT_CODE ON)

set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)


################### TEST
#find_package(GTest REQUIRED)
#include_directories(${GTEST_INCLUDE_DIRS})
#
#add_executable(common_test
#    test/test.cpp
#    test/testFlowIo.cpp
#    test/testMatrixParser.cpp
#    test/testConvertTracksToBinary.cpp
#    test/testTracks.cpp)
#add_dependencies(common_test common)
#message(STATUS OpenCV_LIBS: ${OpenCV_LIBS})
#target_link_libraries(common_test common ${OpenCV_LIBS} gtest gtest_main pthread)