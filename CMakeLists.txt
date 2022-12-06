cmake_minimum_required(VERSION 3.5)

project(BundleAdjustment)
project(ceres-tests)

find_package(Ceres REQUIRED)
#set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS} -lstdc++fs")

include_directories(
    include
)
include_directories(${CMAKE_INCLUDE_PATH} ${CERES_INCLUDE_DIRS})

add_library(readCeresInput examples/readCeresInput.cpp)
target_link_libraries(readCeresInput Ceres::ceres)
add_library(multiViewShapeAdjuster examples/multiViewShapeAdjuster.cpp)
target_link_libraries(multiViewShapeAdjuster Ceres::ceres)
add_library(singleViewPoseAdjuster examples/singleViewPoseAdjuster.cpp)
target_link_libraries(singleViewPoseAdjuster Ceres::ceres)
add_library(singleViewShapeAdjuster examples/singleViewShapeAdjuster.cpp)
target_link_libraries(singleViewShapeAdjuster Ceres::ceres)
add_library(multiViewAdjuster examples/multiViewAdjuster.cpp)
target_link_libraries(multiViewAdjuster Ceres::ceres)

add_executable(ceres_simple examples/ceres_simple.cpp)
target_link_libraries(ceres_simple Ceres::ceres)

# add_executable(singleViewPoseAdjuster examples/singleViewPoseAdjuster.cc)
# target_link_libraries(singleViewPoseAdjuster ${CERES_LIBRARIES})

# add_executable(singleViewShapeAdjuster examples/singleViewShapeAdjuster.cc)
# target_link_libraries(singleViewShapeAdjuster ${CERES_LIBRARIES})

# add_executable(multiViewAdjuster examples/multiViewAdjuster.cc)
# target_link_libraries(multiViewAdjuster ${CERES_LIBRARIES})

add_executable(read_ceres examples/read_ceres.cpp)
target_link_libraries(read_ceres Ceres::ceres readCeresInput)

add_executable(create_ceres examples/create_ceres.cpp)
target_link_libraries(create_ceres Ceres::ceres multiViewShapeAdjuster multiViewAdjuster singleViewShapeAdjuster singleViewPoseAdjuster)
#target_link_libraries(create_ceres Ceres::ceres multiViewShapeAdjuster)
