
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
	svn co http://svn.pointclouds.org/pcl/tags/pcl-1.6.0 $@
	cd svn && patch -p0 < ../patch-1.5.1-2012-04-04

remove_msgs:
	cd svn/common/include; \
	if [ -d std_msgs ]; then \
	mv std_msgs std_msgs_pcl_do_not_use; \
	mv sensor_msgs sensor_msgs_pcl_do_not_use; \
	fi

restore_msgs:
	cd svn/common/include; \
	if [ -d std_msgs_pcl_do_not_use ]; then \
	mv std_msgs_pcl_do_not_use std_msgs; \
	mv sensor_msgs_pcl_do_not_use sensor_msgs; \
	fi

clean: restore_msgs
	cd svn/build && make clean
	-rm built

wipe:
	-rm -rf svn
	-rm built
