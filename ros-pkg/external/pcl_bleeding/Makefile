
built: svn
	cd svn && mkdir -p build
	cd svn/build && cmake ../ -DUSE_ROS=true -DBUILD_TESTS:BOOL=OFF -DBUILD_apps:BOOL=OFF
	cd svn/build && make -j`cat /proc/cpuinfo  | grep processor | wc -l`
	cd svn/common/include; \
	if [ -d std_msgs ]; then \
	mv std_msgs std_msgs_pcl_do_not_use; \
	mv sensor_msgs sensor_msgs_pcl_do_not_use; \
	fi
	touch $@

svn:
	svn co http://svn.pointclouds.org/pcl/tags/pcl-1.6.0 $@
	cd svn && patch -p0 < ../patch-1.6.0-2012-10-28

remove_msgs:
	cd svn/common/include; \
	if [ -d std_msgs ]; then \
	mv std_msgs std_msgs_pcl_do_not_use; \
	mv sensor_msgs sensor_msgs_pcl_do_not_use; \
	fi

restore_msgs:
	if [ -e svn/common/include ]; then \
	cd svn/common/include; \
	if [ -d std_msgs_pcl_do_not_use ]; then \
	mv std_msgs_pcl_do_not_use std_msgs; \
	mv sensor_msgs_pcl_do_not_use sensor_msgs; \
	fi \
	fi

clean: restore_msgs
	if [ -e svn/build ]; then \
	cd svn/build && make clean; \
	fi
	-rm built

wipe:
	-rm -rf svn
	-rm built
