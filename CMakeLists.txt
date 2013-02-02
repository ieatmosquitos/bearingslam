 cmake_minimum_required(VERSION 2.8)
project( g2oBearingSlam )
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

include_directories( ${g2oBearingSlam_SOURCE_DIR}/include )

FIND_PACKAGE(CSparse)
include_directories( ${CSPARSE_INCLUDE_DIR} )

include_directories( $ENV{G2O_ROOT} )
link_directories( $ENV{G2O_ROOT}/lib )


find_package(SFML 1.6 COMPONENTS system window graphics network audio REQUIRED)
include_directories(${SFML_INCLUDE_DIR})

add_executable( LandmarksEstimator src/LandmarksEstimator.cpp src/drawer.cpp )
# add_executable( testRette src/testRette.cpp )
add_executable( MapsMerger src/mapsmerger.cpp )
# add_executable( TrajGenerator src/trajgenerator.cpp)
target_link_libraries( LandmarksEstimator ${Eigen_LIBS} )
find_package(Eigen  REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)

target_link_libraries( MapsMerger ${OpenCV_LIBS} )
target_link_libraries( MapsMerger g2o_core g2o_types_slam2d)

target_link_libraries( LandmarksEstimator ${OpenCV_LIBS} )
target_link_libraries( LandmarksEstimator ${SFML_LIBRARIES} )
target_link_libraries( LandmarksEstimator g2o_core g2o_csparse_extension g2o_parser g2o_solver_csparse g2o_solver_slam2d_linear g2o_stuff g2o_tutorial_slam2d g2o_types_slam2d_addons g2o_types_slam2d)
target_link_libraries( LandmarksEstimator ${CSPARSE_LIBRARY} )

# target_link_libraries( TrajGenerator g2o_core g2o_types_slam2d)

execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory
  ${g2oBearingSlam_SOURCE_DIR}/assets
  ${g2oBearingSlam_BINARY_DIR}/assets/rd
  )