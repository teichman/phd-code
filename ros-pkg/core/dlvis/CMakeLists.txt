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
rosbuild_add_library(dlvis src/lib/dlvis.cpp)
target_link_libraries(dlvis GLU glut)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(display_list_test src/prog/display_list_test.cpp)
target_link_libraries(display_list_test GLU glut)

rosbuild_add_executable(dlvis_test src/prog/dlvis_test.cpp)
target_link_libraries(dlvis_test dlvis)
rosbuild_link_boost(dlvis_test system program_options)

rosbuild_add_executable(pclvis_test src/prog/pclvis_test.cpp)
target_link_libraries(pclvis_test dlvis)

rosbuild_add_executable(dlvis_simple src/prog/dlvis_simple.cpp)
target_link_libraries(dlvis_simple dlvis)
