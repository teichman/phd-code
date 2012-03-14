#!/bin/bash
rosrun camera_calibration cameracalibrator.py --size 6x8 --square 0.1016 image:=/camera/rgb/image_mono camera:=/camera/rgb
rosrun xpl_camera fix_yaml.py `rosparam get camera/driver/rgb_camera_info_url`
