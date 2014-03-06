#!/bin/bash

mkdir -p $1
rosrun rosbag record --topic /foreground --topic /background --split 1024 -j -o ${1}/recording