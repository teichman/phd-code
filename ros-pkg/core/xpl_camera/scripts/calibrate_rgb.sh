#!/bin/bash
CALIB_FILE=`rosparam get camera/driver/rgb_camera_info_url | cut -c8-`
mkdir -p `dirname $CALIB_FILE`
rosrun camera_calibration cameracalibrator.py --size 6x8 --square 0.1016 image:=/camera/rgb/image_mono camera:=/camera/rgb
rosrun xpl_camera fix_yaml.py $CALIB_FILE
