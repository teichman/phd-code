#!/bin/bash
EXPECTED_ARGS=1
E_BADARGS=65
if [ $# -ne $EXPECTED_ARGS ]
then
  echo "Usage: `basename $0` CAMERA_NAME"
  echo "CAMERA_NAME is, i.e., xpl00"
  exit $E_BADARGS
else
  CALIB_FILE=`rospack find xpl_camera`/configuration/$1/rgb.yaml;
  rosrun rgbd_sequence record_stream --vga $CALIB_FILE
fi
