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
  src/lib/transform_validator.cpp
  src/lib/orb_extractor.cpp
  src/lib/orb_matcher.cpp
  src/lib/descriptor_database.cpp
  #  src/lib/calibration_pipeline_orb.cpp
  src/lib/background_modeler.cpp
  src/lib/gaussian_background_modeler.cpp
  src/lib/background_subtractor.cpp
  src/lib/object_extractor.cpp
  src/lib/object_matching_calibrator.cpp
  src/lib/calibration_pipeline_dynamic.cpp
  src/lib/organized_connected_components.cpp
  src/lib/trc_parser.cpp
#  src/lib/mocap_visualizer.cpp
#  src/lib/mocap_detector.cpp
  src/lib/depth_distortion_learner.cpp
  src/lib/velo_to_asus_calibrator.cpp
  src/lib/mean_depth_error.cpp
#  src/lib/slam_visualizer.cpp
  src/lib/frame_aligner.cpp
#  src/lib/loop_closer.cpp
  src/lib/trajectory.cpp
  src/lib/slam_calibrator.cpp
  src/lib/slam_calibration_visualizer.cpp
#  src/lib/slam_lightweight.cpp
  src/lib/primesense_slam.cpp
  src/lib/link_visualizer.cpp
  )

# memory fragmentation is a problem for data quantity experiment?
target_link_libraries(${PROJECT_NAME} tcmalloc)

rosbuild_add_compile_flags(${PROJECT_NAME} -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)
rosbuild_link_boost(${PROJECT_NAME} system program_options)


rosbuild_add_executable(calibrate_sequences src/prog/calibrate_sequences.cpp)
target_link_libraries(calibrate_sequences ${PROJECT_NAME})

rosbuild_add_executable(calibration_viewer src/prog/calibration_viewer.cpp)
target_link_libraries(calibration_viewer ${PROJECT_NAME})

rosbuild_add_executable(calibration_viewer_vis src/prog/calibration_viewer_vis.cpp)
target_link_libraries(calibration_viewer_vis ${PROJECT_NAME})

rosbuild_add_executable(interactive_calibration_viewer src/prog/interactive_calibration_viewer.cpp)
target_link_libraries(interactive_calibration_viewer ${PROJECT_NAME})

rosbuild_add_executable(evaluate src/prog/evaluate.cpp)
target_link_libraries(evaluate ${PROJECT_NAME})

#rosbuild_add_executable(visualize_background_subtraction src/prog/visualize_background_subtraction.cpp)
#target_link_libraries(visualize_background_subtraction ${PROJECT_NAME})

rosbuild_add_executable(offset_maker src/prog/offset_maker.cpp)
target_link_libraries(offset_maker ${PROJECT_NAME})

rosbuild_add_gtest(test_calibration src/test/test_calibration.cpp)
target_link_libraries(test_calibration ${PROJECT_NAME})
rosbuild_add_gtest_build_flags(test_calibration)

#rosbuild_add_executable(visualize_mocap src/prog/visualize_mocap.cpp)
#rosbuild_add_compile_flags(visualize_mocap -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)
#target_link_libraries(visualize_mocap ${PROJECT_NAME})

rosbuild_add_executable(asus_vs_velo src/prog/asus_vs_velo.cpp src/lib/asus_vs_velo_visualizer.cpp)
rosbuild_add_compile_flags(asus_vs_velo -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)
target_link_libraries(asus_vs_velo ${PROJECT_NAME})

#rosbuild_add_executable(test_mocap src/prog/test_mocap.cpp)
#rosbuild_add_compile_flags(test_mocap -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)
#target_link_libraries(test_mocap ${PROJECT_NAME})

rosbuild_add_executable(inspect_depth_image src/prog/inspect_depth_image.cpp)
rosbuild_add_compile_flags(inspect_depth_image -Wno-deprecated -Wno-unused-but-set-variable -Wno-unknown-pragmas)
target_link_libraries(inspect_depth_image ${PROJECT_NAME})

#rosbuild_add_executable(g2o_hello src/prog/g2o_hello.cpp)
#target_link_libraries(g2o_hello)

#rosbuild_add_executable(slam_test src/prog/slam_test.cpp)
#rosbuild_link_boost(slam_test program_options filesystem)
#target_link_libraries(slam_test ${PROJECT_NAME})

#rosbuild_add_executable(test_transforms src/prog/test_transforms.cpp)
#rosbuild_link_boost(test_transforms program_options filesystem)
#target_link_libraries(test_transforms ${PROJECT_NAME})
#
#rosbuild_add_executable(test_loop_closure src/prog/test_loop_closure.cpp)
#rosbuild_link_boost(test_loop_closure program_options filesystem)
#target_link_libraries(test_loop_closure ${PROJECT_NAME})

rosbuild_add_executable(calibrate_slam src/prog/calibrate_slam.cpp)
rosbuild_link_boost(calibrate_slam program_options filesystem)
target_link_libraries(calibrate_slam ${PROJECT_NAME})

rosbuild_add_executable(primesense_slam src/prog/primesense_slam.cpp)
rosbuild_link_boost(primesense_slam program_options filesystem)
target_link_libraries(primesense_slam ${PROJECT_NAME})

#rosbuild_add_executable(slam_run_lightweight src/prog/slam_run_lightweight.cpp)
#rosbuild_link_boost(slam_run_lightweight program_options filesystem)
#target_link_libraries(slam_run_lightweight ${PROJECT_NAME})

rosbuild_add_executable(inspect_pose_graph src/prog/inspect_pose_graph.cpp)
rosbuild_link_boost(inspect_pose_graph program_options filesystem)
target_link_libraries(inspect_pose_graph ${PROJECT_NAME})

rosbuild_add_executable(view_links src/prog/view_links.cpp)
rosbuild_link_boost(view_links program_options filesystem)
target_link_libraries(view_links ${PROJECT_NAME})

rosbuild_add_executable(visualize_frame_alignment src/prog/visualize_frame_alignment.cpp)
rosbuild_link_boost(visualize_frame_alignment program_options filesystem)
target_link_libraries(visualize_frame_alignment ${PROJECT_NAME})

rosbuild_add_executable(evaluate_frame_alignment src/prog/evaluate_frame_alignment.cpp)
rosbuild_link_boost(evaluate_frame_alignment program_options filesystem)
target_link_libraries(evaluate_frame_alignment ${PROJECT_NAME})

rosbuild_add_executable(twiddle_frame_alignment src/prog/twiddle_frame_alignment.cpp)
rosbuild_link_boost(twiddle_frame_alignment program_options filesystem)
target_link_libraries(twiddle_frame_alignment ${PROJECT_NAME})

rosbuild_add_executable(avv_multiview_calibrator src/prog/avv_multiview_calibrator.cpp src/lib/asus_vs_velo_visualizer.cpp src/lib/avv_multiview_model.cpp)
rosbuild_link_boost(avv_multiview_calibrator program_options filesystem)
target_link_libraries(avv_multiview_calibrator ${PROJECT_NAME})

rosbuild_add_executable(visualize_slam_calibration src/prog/visualize_slam_calibration.cpp)
rosbuild_link_boost(visualize_slam_calibration program_options filesystem)
target_link_libraries(visualize_slam_calibration ${PROJECT_NAME})

rosbuild_add_executable(evaluate_slam_calibration src/prog/evaluate_slam_calibration.cpp)
rosbuild_link_boost(evaluate_slam_calibration program_options filesystem)
target_link_libraries(evaluate_slam_calibration ${PROJECT_NAME})

rosbuild_add_executable(evaluate_quantity src/prog/evaluate_quantity.cpp)
rosbuild_link_boost(evaluate_quantity program_options filesystem)
target_link_libraries(evaluate_quantity ${PROJECT_NAME})

rosbuild_add_executable(undistortion_time_experiment src/prog/undistortion_time_experiment.cpp)
rosbuild_link_boost(undistortion_time_experiment program_options filesystem)
target_link_libraries(undistortion_time_experiment ${PROJECT_NAME})

rosbuild_add_executable(focal_length_experiment src/prog/focal_length_experiment.cpp)
rosbuild_link_boost(focal_length_experiment program_options filesystem)
target_link_libraries(focal_length_experiment ${PROJECT_NAME})

rosbuild_add_executable(visualize_intrinsics src/prog/visualize_intrinsics.cpp)
rosbuild_link_boost(visualize_intrinsics program_options filesystem)
target_link_libraries(visualize_intrinsics ${PROJECT_NAME})

rosbuild_add_executable(iterative_calibration_experiment src/prog/iterative_calibration_experiment.cpp)
rosbuild_link_boost(iterative_calibration_experiment program_options filesystem)
target_link_libraries(iterative_calibration_experiment ${PROJECT_NAME})

rosbuild_add_executable(planarity_experiment src/prog/planarity_experiment.cpp)
rosbuild_link_boost(planarity_experiment program_options filesystem)
target_link_libraries(planarity_experiment ${PROJECT_NAME})

rosbuild_add_executable(view_stream src/prog/view_stream.cpp)
rosbuild_link_boost(view_stream program_options filesystem)
target_link_libraries(view_stream ${PROJECT_NAME})

rosbuild_add_executable(visualize_map_depth_estimation src/prog/visualize_map_depth_estimation.cpp)
rosbuild_link_boost(visualize_map_depth_estimation program_options filesystem)
target_link_libraries(visualize_map_depth_estimation ${PROJECT_NAME})

rosbuild_add_executable(generate_clams_video src/prog/generate_clams_video.cpp)
rosbuild_link_boost(generate_clams_video program_options filesystem)
target_link_libraries(generate_clams_video ${PROJECT_NAME})
