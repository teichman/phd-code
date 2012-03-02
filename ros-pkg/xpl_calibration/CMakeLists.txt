cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/xpl_calibrator.cpp
  src/plane_finder.cpp
  src/color_wheel.cpp
  src/transform_validator.cpp
  src/orb_extractor.cpp
  src/xpl_calibrator_orb.cpp
  src/descriptor_database.cpp
  )
rosbuild_link_boost(${PROJECT_NAME} system)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(calibrate_sequences src/calibrate_sequences.cpp)
target_link_libraries(calibrate_sequences ${PROJECT_NAME})

rosbuild_add_executable(calibration_viewer src/calibration_viewer.cpp)
target_link_libraries(calibration_viewer ${PROJECT_NAME})
