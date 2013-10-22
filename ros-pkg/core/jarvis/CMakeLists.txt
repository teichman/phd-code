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
rosbuild_add_library(jarvis_core
  src/lib/jarvis.cpp
  src/lib/tracker.cpp
  src/lib/track_dataset_assembler.cpp
  src/lib/blob_serializer.cpp
  src/lib/blob_view.cpp
  src/lib/descriptor_pipeline.cpp
  src/lib/pods.cpp
  src/lib/jarvis_twiddler.cpp
  src/lib/inductor.cpp
  )
rosbuild_add_compile_flags(jarvis_core -std=c++0x)
target_link_libraries(jarvis_core tcmalloc)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

#rosbuild_add_executable(visualize_detections src/prog/visualize_detections.cpp)
#target_link_libraries(visualize_detections jarvis)

rosbuild_add_executable(jarvis src/prog/jarvis.cpp)
target_link_libraries(jarvis jarvis_core)
rosbuild_link_boost(jarvis program_options)

rosbuild_add_executable(visualize_augmented_td src/prog/visualize_augmented_td.cpp)
target_link_libraries(visualize_augmented_td jarvis_core)
rosbuild_link_boost(visualize_augmented_td program_options)

rosbuild_add_executable(update_descriptors src/prog/update_descriptors.cpp)
target_link_libraries(update_descriptors jarvis_core)
rosbuild_link_boost(update_descriptors program_options)

rosbuild_add_executable(induct src/prog/induct.cpp)
target_link_libraries(induct jarvis_core)
rosbuild_link_boost(induct program_options)

rosbuild_add_executable(evaluate src/prog/evaluate.cpp)
target_link_libraries(evaluate jarvis_core)
rosbuild_link_boost(evaluate program_options)

rosbuild_add_executable(bias_variance src/prog/bias_variance.cpp)
target_link_libraries(bias_variance jarvis_core)
rosbuild_link_boost(bias_variance program_options)

rosbuild_add_executable(baseline_unfair src/prog/baseline_unfair.cpp)
target_link_libraries(baseline_unfair jarvis_core)
rosbuild_link_boost(baseline_unfair program_options)

rosbuild_add_executable(cross_evaluate src/prog/cross_evaluate.cpp)
target_link_libraries(cross_evaluate jarvis_core)
rosbuild_link_boost(cross_evaluate program_options)

rosbuild_add_executable(twiddle src/prog/twiddle.cpp)
target_link_libraries(twiddle jarvis_core)
rosbuild_add_compile_flags(twiddle -std=c++0x)
rosbuild_link_boost(twiddle program_options)

rosbuild_add_executable(pcd_viewer src/prog/pcd_viewer.cpp)
target_link_libraries(pcd_viewer vtkCommon vtkIO vtkFiltering vtkHybrid vtkRendering)
rosbuild_link_boost(pcd_viewer program_options)

rosbuild_add_gtest(test_jarvis src/test/test_jarvis.cpp)
target_link_libraries(test_jarvis jarvis_core)

