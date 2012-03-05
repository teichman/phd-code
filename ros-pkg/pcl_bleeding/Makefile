
built: svn
	cd svn && mkdir -p build
	cd svn/build && cmake ../ -DUSE_ROS=true
	cd svn/build && make -j`cat /proc/cpuinfo  | grep processor | wc -l`
	cd svn/common/include; \
	if [ -d std_msgs ]; then \
	mv std_msgs std_msgs_pcl_do_not_use; \
	mv sensor_msgs sensor_msgs_pcl_do_not_use; \
	fi
	touch $@

svn:
	svn co -r3516 http://svn.pointclouds.org/pcl/trunk $@
	cd svn/common/include/pcl && patch < ../../../../patch
	cd svn/io/src/ && patch < ../../../openni_grabber.cpp.patch
	cd svn/common/include/pcl/common/impl && patch < ../../../../../../transformation_from_correspondences.hpp.patch

clean:
	cd svn/build && make clean
	cd svn/common/include; \
	if [ -d std_msgs_pcl_do_not_use ]; then \
	mv std_msgs_pcl_do_not_use std_msgs; \
	mv sensor_msgs_pcl_do_not_use sensor_msgs; \
	fi
	-rm built

wipe:
	-rm -rf svn
	-rm built
