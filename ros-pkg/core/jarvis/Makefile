ROS_PARALLEL_JOBS := -j$$(if [ -n "`grep ARM /proc/cpuinfo`" ]; then echo 2; else cat /proc/cpuinfo | grep processor | wc -l; fi)
include $(shell rospack find mk)/cmake.mk