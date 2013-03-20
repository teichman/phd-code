#!/bin/bash

if [ ! $# -eq 2 ]
then
    echo "Usage: record_sequence SENSORID DESCRIPTIVENAME"
    echo "  SENSORID is, for example, xpl01."
else
    DIR=`date +%F`_$1_$2
    if [ -e $DIR ]
    then
        echo Directory $DIR already exists.  Exiting.
    else
        echo NTP update.
        sudo ntpdate -b ntp.ubuntu.com
        mkdir $DIR
        cd $DIR
        `rospack find pcl_trunk`/release/bin/pcl_openni_image
        #cp `rospack find xpl_calibration`/distortion_models/${2} ./distortion_model
        cd -
    fi
fi
