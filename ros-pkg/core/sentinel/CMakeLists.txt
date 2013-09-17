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
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/lib/sentinel.cpp
  src/lib/background_model.cpp
  src/lib/reconstructor.cpp
  )
rosbuild_add_compile_flags(sentinel -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)  # -DJARVIS_DEBUG
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} filesystem program_options)

#rosbuild_add_executable(watch src/prog/watch.cpp)
#target_link_libraries(watch ${PROJECT_NAME})
#rosbuild_add_compile_flags(watch -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas -DJARVIS_DEBUG)

rosbuild_add_executable(stream src/prog/stream.cpp)
target_link_libraries(stream ${PROJECT_NAME})
rosbuild_add_compile_flags(stream -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)  # -DJARVIS_PROFILE -DJARVIS_DEBUG

rosbuild_add_executable(estimate_noise src/prog/estimate_noise.cpp)
