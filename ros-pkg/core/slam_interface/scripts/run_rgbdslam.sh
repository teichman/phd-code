#!/bin/bash
SEQ_DIR=$1
PAUSE_TIME=1
MAX_DISTANCE=2 #meters
MY_PACKAGE=slam_interface
sleep 5
rosservice call /rgbdslam/ros_ui reset
rosservice call /rgbdslam/ros_ui_b pause false
sleep 10
rosrun ${MY_PACKAGE} publish_sequence.sh \
  ${SEQ_DIR} ${PAUSE_TIME} ${MAX_DISTANCE}
rosservice call /rgbdslam/ros_ui_b pause true
#sleep 5
rosservice call /rgbdslam/ros_ui save_all
sleep 5
rosservice call /rgbdslam/ros_ui save_individual
rosservice call /rgbdslam/ros_ui save_trajectory
sleep 10
rosservice call /rgbdslam/ros_ui reset
PCD_DIR=${SEQ_DIR}_RGBDSLAM
mkdir -p ${PCD_DIR}
RGBDSLAM_DIR=`rospack find rgbdslam`
mv ${RGBDSLAM_DIR}/bin/quicksave.pcd ${PCD_DIR}/all.pcd
mv ${RGBDSLAM_DIR}/bin/trajectory_estimate.txt ${PCD_DIR}
for f in `ls ${RGBDSLAM_DIR}/bin | grep quicksave.pcd_`; do
  NEWNAME=`echo ${f} | sed "s/quicksave.pcd_/frame_/g"`
  mv ${RGBDSLAM_DIR}/bin/${f} ${PCD_DIR}/${NEWNAME}
done
#Filter combined cloud into reasonable size
rosrun pcl_bleeding pcl_voxel_grid -leaf 0.02,0.02,0.02 ${PCD_DIR}/all.pcd ${PCD_DIR}/all_filtered.pcd
