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

rosbuild_add_boost_directories()
#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/lib/descriptor_pipeline.cpp)

#target_link_libraries(${PROJECT_NAME} another_library)

rosbuild_add_executable(optical_flow_experiment src/prog/optical_flow_experiment.cpp)
rosbuild_add_executable(dump src/prog/dump.cpp)
rosbuild_add_executable(generate_unlabeled_dataset src/prog/generate_unlabeled_dataset.cpp)
rosbuild_link_boost(generate_unlabeled_dataset program_options filesystem)

rosbuild_add_executable(lkdemo src/prog/lkdemo.cpp)

rosbuild_add_executable(cross_evaluate src/prog/cross_evaluate.cpp)
rosbuild_link_boost(cross_evaluate program_options filesystem)

rosbuild_add_executable(predict src/prog/predict.cpp)
target_link_libraries(predict ${PROJECT_NAME})
rosbuild_link_boost(predict program_options filesystem)

#target_link_libraries(example ${PROJECT_NAME})
