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
rosbuild_add_library(rgbd_sequence
  src/rgbd_sequence.cpp
  src/compact_sequence.cpp
  src/stream_sequence.cpp
  src/recorder.cpp
  src/compact_recorder.cpp
  src/stream_recorder.cpp
  src/publisher.cpp
  )
rosbuild_link_boost(rgbd_sequence system)
rosbuild_add_compile_flags(rgbd_sequence -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(record src/record.cpp)
target_link_libraries(record rgbd_sequence)
rosbuild_add_compile_flags(record -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(record_compact src/record_compact.cpp)
target_link_libraries(record_compact rgbd_sequence)
rosbuild_add_compile_flags(record_compact -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(record_stream src/record_stream.cpp)
target_link_libraries(record_stream rgbd_sequence)
rosbuild_add_compile_flags(record_stream -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(publish src/publish.cpp)
target_link_libraries(publish rgbd_sequence)
rosbuild_add_compile_flags(publish -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(view_compact src/view_compact.cpp)
target_link_libraries(view_compact rgbd_sequence)
rosbuild_add_compile_flags(view_compact -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(view_stream src/view_stream.cpp)
target_link_libraries(view_stream rgbd_sequence)
rosbuild_add_compile_flags(view_stream -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_gtest(test_stream src/test_stream.cpp)
target_link_libraries(test_stream rgbd_sequence)
rosbuild_add_gtest_build_flags(test_stream)
