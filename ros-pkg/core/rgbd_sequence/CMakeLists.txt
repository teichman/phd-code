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
  src/lib/rgbd_sequence.cpp
  src/lib/stream_sequence.cpp
  src/lib/stream_recorder.cpp
#  src/lib/publisher.cpp
  src/lib/projector.cpp
  src/lib/vis_wrapper.cpp
  src/lib/tube_measurer.cpp
  )
rosbuild_link_boost(rgbd_sequence system program_options)
rosbuild_add_compile_flags(rgbd_sequence -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(record_stream src/prog/record_stream.cpp)
target_link_libraries(record_stream rgbd_sequence)
rosbuild_add_compile_flags(record_stream -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

# OpenCV version conflict because of the bridge.
#rosbuild_add_executable(publish src/prog/publish.cpp)
#target_link_libraries(publish rgbd_sequence)
#rosbuild_add_compile_flags(publish -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(view_stream src/prog/view_stream.cpp)
target_link_libraries(view_stream rgbd_sequence)
rosbuild_add_compile_flags(view_stream -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(offset_timestamps src/prog/offset_timestamps.cpp)
target_link_libraries(offset_timestamps rgbd_sequence)
rosbuild_add_compile_flags(offset_timestamps -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(pcd_series_view src/prog/pcd_series_view.cpp)
target_link_libraries(pcd_series_view rgbd_sequence)
rosbuild_add_compile_flags(pcd_series_view -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_gtest(test_stream src/test/test_stream.cpp)
target_link_libraries(test_stream rgbd_sequence)
rosbuild_add_gtest_build_flags(test_stream)

rosbuild_add_executable(get_primesense_info src/prog/get_primesense_info.cpp)
rosbuild_add_compile_flags(get_primesense_info -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)

rosbuild_add_executable(visualize_intrinsics src/prog/visualize_intrinsics.cpp)
rosbuild_add_compile_flags(visualize_intrinsics -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)
target_link_libraries(visualize_intrinsics rgbd_sequence)

rosbuild_add_executable(intrinsics_search src/prog/intrinsics_search.cpp)
rosbuild_add_compile_flags(intrinsics_search -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)
target_link_libraries(intrinsics_search rgbd_sequence)
