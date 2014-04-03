ROS_PARALLEL_JOBS := -j`if [ $HOSTTYPE = "arm" ]; then echo 2 ; else cat /proc/cpuinfo  | grep processor | wc -l ; fi`
include $(shell rospack find mk)/cmake.mk
