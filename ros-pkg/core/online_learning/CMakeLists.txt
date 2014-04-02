cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(/usr/include/vtk-5.8/)

rosbuild_add_boost_directories()


########################################
# Libraries
########################################

rosbuild_add_library(online_learning
  src/lib/dataset.cpp
  src/lib/schedulers.cpp
  src/lib/synthetic_data_generator.cpp
  src/lib/classifier.cpp
  src/lib/grid_classifier.cpp
  src/lib/evaluator.cpp
  src/lib/training_buffer.cpp
  src/lib/tbssl.cpp
  src/lib/collage_generator.cpp
  src/lib/track_dataset_visualizer.cpp
  src/lib/cross_evaluator.cpp
  src/lib/gc_broadcaster.cpp
  src/lib/clusterer.cpp
  src/lib/instance_serializer.cpp
  )
rosbuild_add_compile_flags(online_learning -Wno-deprecated-declarations)
target_link_libraries(online_learning tcmalloc)
rosbuild_link_boost(online_learning program_options system)

# This library just initializes Instance::custom_serializer_
# to EmptyCustomSerializer.
rosbuild_add_library(online_learning_ecs
  src/lib/ecs.cpp
  )
rosbuild_add_compile_flags(online_learning_ecs -Wno-deprecated-declarations)

# This library just initializes Instance::custom_serializer_
# to PassthroughCustomSerializer.
rosbuild_add_library(online_learning_pcs
  src/lib/pcs.cpp
  )
rosbuild_add_compile_flags(online_learning_pcs -Wno-deprecated-declarations)

rosbuild_add_library(online_learning_pcd_serializer
  src/lib/use_pcd_serializer.cpp
  )

########################################
# GTests
########################################

rosbuild_add_gtest(test_custom_serializer src/test/test_custom_serializer.cpp)
target_link_libraries(test_custom_serializer online_learning online_learning_ecs)
  
rosbuild_add_gtest(test_dataset src/test/test_dataset.cpp)
target_link_libraries(test_dataset online_learning online_learning_ecs)

rosbuild_add_gtest(test_tbssl src/test/test_tbssl.cpp TIMEOUT 99999999999999)
target_link_libraries(test_tbssl online_learning online_learning_ecs)

rosbuild_add_gtest(test_grid_classifier src/test/test_grid_classifier.cpp)
target_link_libraries(test_grid_classifier online_learning online_learning_ecs)

rosbuild_add_gtest(test_collage src/test/test_collage.cpp)
target_link_libraries(test_collage online_learning online_learning_ecs)

#rosbuild_add_gtest(test_augmented_instances src/test/test_augmented_instances.cpp)
#target_link_libraries(test_augmented_instances augmented_instance_experiment)

########################################
# Executables
########################################

# -- These programs do not use custom data in any way,
#    so they use EmptyCustomSerializer.  Custom data is not
#    read from disk.

rosbuild_add_executable(baseline_unfair src/program/baseline_unfair.cpp)
target_link_libraries(baseline_unfair online_learning online_learning_ecs)

rosbuild_add_executable(baseline src/program/baseline.cpp)
target_link_libraries(baseline online_learning online_learning_ecs)

rosbuild_add_executable(precision src/program/precision.cpp)
target_link_libraries(precision online_learning online_learning_ecs)

rosbuild_add_executable(plot_descriptor src/program/plot_descriptor.cpp)
target_link_libraries(plot_descriptor online_learning online_learning_ecs)

rosbuild_add_executable(plot_classifier src/program/plot_classifier.cpp)
target_link_libraries(plot_classifier online_learning online_learning_ecs)

rosbuild_add_executable(reset_classifier src/program/reset_classifier.cpp)
target_link_libraries(reset_classifier online_learning online_learning_ecs)

rosbuild_add_executable(plot_grid src/program/plot_grid.cpp)
target_link_libraries(plot_grid online_learning online_learning_ecs)

rosbuild_add_executable(evaluate_supervised src/program/evaluate_supervised.cpp)
target_link_libraries(evaluate_supervised online_learning online_learning_ecs)

rosbuild_add_executable(tbssl_plots src/program/tbssl_plots.cpp)
target_link_libraries(tbssl_plots online_learning online_learning_ecs)

rosbuild_add_executable(leak src/program/leak.cpp)
target_link_libraries(leak online_learning online_learning_ecs)

rosbuild_add_executable(stat src/program/stat.cpp)
target_link_libraries(stat online_learning online_learning_ecs)

rosbuild_add_executable(learner_status src/program/learner_status.cpp)
target_link_libraries(learner_status online_learning online_learning_ecs)

rosbuild_add_executable(train_post_classifier src/program/train_post_classifier.cpp)
target_link_libraries(train_post_classifier online_learning online_learning_ecs)

rosbuild_add_executable(evaluate_loo src/program/evaluate_loo.cpp)
target_link_libraries(evaluate_loo online_learning online_learning_ecs)

# -- These programs use PassthroughCustomSerializer so that custom data will be
#    read and then saved again without knowing what it is.

#rosbuild_add_executable(unlabel src/program/unlabel.cpp)
#target_link_libraries(unlabel online_learning online_learning_pcs)

rosbuild_add_executable(label src/program/label.cpp)
target_link_libraries(label online_learning online_learning_pcs)

rosbuild_add_executable(apply_cmap src/program/apply_cmap.cpp)
target_link_libraries(apply_cmap online_learning online_learning_pcs)

rosbuild_add_executable(filter_errors src/program/filter_errors.cpp)
target_link_libraries(filter_errors online_learning online_learning_pcs)

rosbuild_add_executable(partition_td src/program/partition_td.cpp)
target_link_libraries(partition_td online_learning online_learning_pcs)

rosbuild_add_executable(merge_tds src/program/merge_tds.cpp)
target_link_libraries(merge_tds online_learning online_learning_pcs)

rosbuild_add_executable(set_importance src/program/set_importance.cpp)
target_link_libraries(set_importance online_learning online_learning_pcs)

rosbuild_add_executable(sample_random_frames src/program/sample_random_frames.cpp)
target_link_libraries(sample_random_frames online_learning online_learning_pcs)

rosbuild_add_executable(sample_random_tracks src/program/sample_random_tracks.cpp)
target_link_libraries(sample_random_tracks online_learning online_learning_pcs)

rosbuild_add_executable(filter_tracks src/program/filter_tracks.cpp)
target_link_libraries(filter_tracks online_learning online_learning_pcs)

rosbuild_add_executable(classify src/program/classify.cpp)
target_link_libraries(classify online_learning online_learning_pcs)

# -- Programs that need to use PCDSerializer.
#    This content should maybe be moved out to another package.

rosbuild_add_executable(start_grind src/program/start_grind.cpp)
target_link_libraries(start_grind online_learning online_learning_pcd_serializer)

rosbuild_add_executable(resume_grind src/program/resume_grind.cpp)
target_link_libraries(resume_grind online_learning online_learning_pcd_serializer)

rosbuild_add_executable(visualize_augmented_td src/program/visualize_augmented_td.cpp)
target_link_libraries(visualize_augmented_td online_learning online_learning_pcd_serializer)

rosbuild_add_executable(generate_group_induction_video src/program/generate_group_induction_video.cpp)
target_link_libraries(generate_group_induction_video online_learning online_learning_pcd_serializer vtkCommon vtkRendering vtkFiltering)

# -- Other

rosbuild_add_executable(nprocs src/program/nprocs.cpp)

