cmake_minimum_required( VERSION 2.8.1 )
project(bundle_physics)

add_definitions(-DPICOJSON_USE_INT64)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

FIND_PACKAGE(OpenCV REQUIRED cudabgsegm highgui imgproc core
    HINTS ${OpenCV_HINTS_DIR})

set(smash_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external
    ${vis_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    #${CMAKE_CURRENT_SOURCE_DIR}/include
    #${CMAKE_CURRENT_SOURCE_DIR}/../funcAcq/include
    #${PHYSACQ_DIR}/../aligner/include/
    #${PHYSACQ_DIR}/include/
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
INCLUDE_DIRECTORIES(${smash_INCLUDE_DIRS})
message(STATUS "${PROJECT_NAME}_INCLUDE_DIRS ${smash_INCLUDE_DIRS}")


SET(PHYS_SRC_LIST
    src/main.cpp
    src/annot/cuboid.cpp
    src/annot/testQuat.cpp
    src/phys/bundleWithPhysics.cpp
    src/phys/physOutput.cpp
    src/phys/bundleWithPhysicsResult.cpp
    src/phys/infMass.cpp
    src/phys/vis/visPhys.cpp
    src/phys/vis/vis.cpp
    src/phys/io/cuboidIo.cpp
    src/phys/io/weightsIo.cpp
    src/phys/energyTerms/gravityTerm.cpp
    src/phys/energyTerms/pointTerm.cpp
    src/phys/energyTerms/impulseTerm.cpp
    src/phys/energyTerms/conservationTerm.cpp
    src/phys/energyTerms/poseTerm.cpp
    src/phys/energyTerms/sfmImpulseTerm.cpp
    src/phys/initialize/optParabolas.cpp
    src/phys/initialize/assignments.cpp
    src/phys/initialize/solve.cpp
    src/phys/initialize/initialize.cpp
    src/phys/initialize/pointTerm.cpp
    src/phys/initialize/fit2dParabola.cpp
    src/phys/initialize/blobExtractor.cpp
    src/phys/initialize/bgFgExtractor.cpp
    src/phys/initialize/ransacParabolas.cpp
    src/phys/energyTerms/parabolaTerm.cpp
    src/phys/io/parabolaIo.cpp
    src/phys/preprocessCuboids.cpp
    src/phys/energyTerms/corTerm.cpp
    src/phys/initialize/parabola2d.cpp
    src/phys/energyTerms/parabola2dTerm.cpp
    src/quatTest.cpp
    #src/phys/tripleColl.cpp
    src/phys/physIndexer.cpp)

SET(PHYS_H_LIST
    #${PHYSACQ_DIR}/include/physacq/simulation/rigidBody.h
    include/tracking/annot/cuboid.h
    include/tracking/annot/impl/cuboid.hpp
    include/tracking/annot/cuboidFwDecl.h
    include/tracking/phys/bundleWithPhysics.h
    include/tracking/phys/physIndexer.h
    include/tracking/phys/energyTerms/impl/poseTerm.hpp
    include/tracking/phys/ceresUtil.h
    include/tracking/phys/trackingLogger.h
    include/tracking/phys/typedefs.h
    include/tracking/phys/parabola.h
    include/tracking/phys/bundleWithPhysicsResult.h
    include/tracking/phys/consts.h
    include/tracking/phys/weights.h
    include/tracking/phys/partUtil.h
    include/tracking/phys/infMass.h
    include/tracking/phys/physUtils.h
    include/tracking/phys/typedefsGeometry.h
    include/tracking/phys/inertiaEstimation.hpp
    include/tracking/phys/initialize/assignments.h
    include/tracking/phys/initialize/initialize.h
    include/tracking/phys/initialize/pointTerm.h
    include/tracking/phys/energyTerms/impulseTerm.h
    include/tracking/phys/energyTerms/impl/impulseTerm.hpp
    include/tracking/phys/energyTerms/conservationTerm.h
    include/tracking/phys/energyTerms/impl/conservationTerm.hpp
    include/tracking/phys/energyTerms/poseTerm.h
    include/tracking/phys/energyTerms/impl/conservationTerm.hpp
    include/tracking/phys/energyTerms/restrictCollPointTerm.h
    include/tracking/phys/energyTerms/impl/restrictCollPointTerm.hpp
    include/tracking/phys/energyTerms/pointTerm.h
    include/tracking/phys/energyTerms/impl/pointTerm.hpp
    include/tracking/phys/energyTerms/impl/compactnessTerm.hpp
    include/tracking/phys/energyTerms/impl/batchPointTerm.hpp
    include/tracking/phys/energyTerms/impl/minMomentumTerm.h
    include/tracking/phys/energyTerms/impl/gravityTerm.hpp
    include/tracking/phys/energyTerms/sfmImpulseTerm.h
    include/tracking/phys/energyTerms/impl/parabolaTerm.hpp
    include/tracking/phys/io/parabolaIo.h
    include/tracking/phys/io/cuboidIo.h
    include/tracking/phys/io/weightsIo.h
    include/tracking/phys/physFunctorInfo.h
    include/tracking/phys/physFunctorInfoFwDecl.h
    include/tracking/phys/physOutput.h
    include/tracking/phys/vis/impl/vis.hpp
    include/tracking/phys/vis/visPhys.h
    include/tracking/unique_ptr.h
    include/tracking/phys/energyTerms/compactnessTerm.h
    include/tracking/phys/initialize/fitParabolas.h
    include/tracking/phys/typdefsTermTypes.h
    include/tracking/phys/io/impl/cuboidIo.hpp
    include/tracking/phys/initialize/bgFgExtractor.h
    include/tracking/phys/initialize/parabola2d.h
    include/tracking/phys/energyTerms/parabolaTerm.h
    include/tracking/phys/energyTerms/impl/CoRBoundsTerm.hpp
    include/tracking/phys/energyTerms/CoRBoundsTerm.h
    include/tracking/phys/preprocessCuboids.h
    include/tracking/phys/initialize/circle.h
    include/tracking/phys/energyTerms/parabola2dTerm.h
    include/tracking/phys/energyTerms/impl/parabola2dTerm.hpp
    include/tracking/phys/tripleColl.h
    include/tracking/phys/physProblem.h)

add_executable(smash ${PHYS_SRC_LIST} ${PHYS_H_LIST})
target_link_libraries(smash
    common
    vis
    ${BUNDLER_LIBRARIES}
    ${GLOG_LIBRARY}
    ${CERES_LIBRARIES}
    -L${OpenCV_DIR}/../../lib/ -lopencv_cudalegacy
    opencv_xfeatures2d
    ${OpenCV_LIBS}
    #${SUITESPARSE_LIBRARIES}
    ${VTK_LIBRARIES}
)

