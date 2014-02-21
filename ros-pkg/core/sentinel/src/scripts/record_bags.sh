#!/bin/bash

mkdir -p $1
rosrun rosbag record -a --split 1024 -j -o ${1}/recording