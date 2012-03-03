
built: svn
	cd svn && mkdir -p build
	cd svn/build && cmake ../ -DCMAKE_BUILD_TYPE=Release -DUSE_ROS=true
	cd svn/build && make -j`cat /proc/cpuinfo  | grep processor | wc -l`
	cd svn/common/include; \
	if [ -d std_msgs ]; then \
	mv std_msgs std_msgs_pcl_do_not_use; \
	mv sensor_msgs sensor_msgs_pcl_do_not_use; \
	fi
	touch $@

svn:
	svn co -r4900 http://svn.pointclouds.org/pcl/trunk $@
	cd $@ && patch -p0 < ../2012-03-02_patch


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
