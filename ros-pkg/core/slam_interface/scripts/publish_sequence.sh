#!/bin/bash
SEQ_DIR=$1
PAUSE_TIME=$2
MAX_DISTANCE=$3
SUBSAMPLE=$4
PUBLISH_PKG=slam_interface
rosrun slam_interface publish \
  ${SEQ_DIR} ${PAUSE_TIME} ${MAX_DISTANCE} ${SUBSAMPLE}\
  image_out:=/camera/rgb/image_color \
  image_info_out:=/camera/rgb/camera_info \
  depth_out:=/camera/depth/image \
  depth_info_out:=/camera/depth/camera_info \
  points_out:=/camera/points2foo
