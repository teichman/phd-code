.SECONDARY:

CUDA_BINDIR := /tmp/gcc-4.4
 PCL_VERSION := 8755

C_COMPILER:=/usr/bin/gcc
CXX_COMPILER:=/usr/bin/g++

default: built_against_ros

built_against_ros: svn svn/PATCHED_KINFU svn/PATCHED_USE_ROS
	mkdir -p msgs_tmp
	for d in `ls svn/common/include | grep msgs`; do \
		echo Moving $$d; \
	  mv svn/common/include/$${d} msgs_tmp; \
  done
	cd svn && mkdir -p build_against_ros
	mkdir -p ${CUDA_BINDIR}
	cd ${CUDA_BINDIR} && ln -sf /usr/bin/gcc-4.6 gcc && ln -sf /usr/bin/g++-4.6 g++ && ln -sf /usr/bin/gcc-4.6 cc && ln -sf /usr/bin/g++-4.6 c++
	cd svn/build_against_ros && cmake ../ \
	-DCMAKE_C_COMPILER:FILEPATH=${C_COMPILER} \
	-DCMAKE_CXX_COMPILER:FILEPATH=${CXX_COMPILER} \
	-DUSE_ROS:BOOL=ON \
	-DBUILD_CUDA:BOOL=OFF \
	-DBUILD_GPU:BOOL=OFF \
	-DBUILD_global_tests:BOOL=OFF \
	-DBUILD_apps:BOOL=ON \
	-DBUILD_gpu_surface:BOOL=ON \
	-DCUDA_NVCC_FLAGS:="--compiler-bindir=${CUDA_BINDIR}" \
	-DCMAKE_CXX_FLAGS:STRING="-Wl,--no-as-needed -Wno-invalid-offsetof -Wno-deprecated-declarations `rospack depends1 pcl_trunk | xargs -I {} ../../rospack_to_flags {}`" \
	-DCMAKE_C_FLAGS:=" -Wno-invalid-offsetof -Wno-deprecated-declarations `rospack depends1 pcl_trunk | xargs -I {} ../../rospack_to_flags {}`" \
	-DCMAKE_EXE_LINKER_FLAGS="-Wl,--no-as-needed `rospack depends1 pcl_trunk | xargs -I {} ../../rospack_to_linker_flags {}` -Wl,-rpath,`rospack find pcl_trunk`/release_against_ros/lib" \
	-DCMAKE_BUILD_TYPE:="Release" \
	-DBUILD_examples:BOOL=OFF \
	-DCMAKE_INSTALL_PREFIX:=`rospack find pcl_trunk`/release_against_ros
	cd svn/build_against_ros && make -j4 && make install
	rm -rf ${CUDA_BINDIR}
	for d in `echo msgs_tmp/*`; do \
		mv $$d svn/common/include; \
	done
	#Gross, but forcing it to export these
	cp svn/common/include/pcl/PolygonMesh.h release_against_ros/include/pcl-1.7/pcl
	cp svn/common/include/pcl/PointIndices.h release_against_ros/include/pcl-1.7/pcl
	cp svn/common/include/pcl/ModelCoefficients.h release_against_ros/include/pcl-1.7/pcl
	cp svn/common/include/pcl/Vertices.h release_against_ros/include/pcl-1.7/pcl
	# Make rospack only look in the installed location
	for f in release*; do \
		touch $${f}/rospack_nosubdirs; \
	done
	touch svn/rospack_nosubdirs
	rm -f release_against_ros/rospack_nosubdirs
	touch $@

# was 5600
built: svn svn/PATCHED_KINFU
	cd svn && mkdir -p build
	mkdir -p ${CUDA_BINDIR}
	cd ${CUDA_BINDIR} && ln -sf /usr/bin/gcc-4.4 gcc && ln -sf /usr/bin/g++-4.4 g++
	cd svn/build && cmake ../ \
	-DCMAKE_C_COMPILER:FILEPATH=${C_COMPILER} \
	-DCMAKE_CXX_COMPILER:FILEPATH=${CXX_COMPILER} \
	-DUSE_ROS:BOOL=OFF \
	-DBUILD_CUDA:BOOL=ON \
	-DBUILD_GPU:BOOL=ON \
	-DBUILD_global_tests:BOOL=OFF \
	-DBUILD_apps:BOOL=ON \
	-DBUILD_gpu_surface:BOOL=ON \
	-DCUDA_NVCC_FLAGS:="--compiler-bindir=${CUDA_BINDIR};-w" \
	-DCMAKE_CXX_FLAGS:=" -Wno-invalid-offsetof -Wno-deprecated-declarations" \
	-DCMAKE_BUILD_TYPE:="Release" \
	-DBUILD_examples:BOOL=OFF \
	-DCMAKE_INSTALL_PREFIX:=`rospack find pcl_trunk`/release \
	-DADD_DEPRECATED_ROS_INCLUDES=1
	cd svn/build && make -j4 && make install
	rm -rf ${CUDA_BINDIR}
	touch $@

built_debug: svn svn/PATCHED_KINFU
	cd svn && mkdir -p build_debug
	mkdir -p ${CUDA_BINDIR}
	cd ${CUDA_BINDIR} && ln -sf /usr/bin/gcc-4.4 gcc && ln -sf /usr/bin/g++-4.4 g++
	cd svn/build_debug && cmake ../ \
	-DCMAKE_C_COMPILER:FILEPATH=${C_COMPILER} \
	-DCMAKE_CXX_COMPILER:FILEPATH=${CXX_COMPILER} \
	-DUSE_ROS:BOOL=OFF \
	-DBUILD_CUDA:BOOL=ON \
	-DBUILD_GPU:BOOL=ON \
	-DBUILD_global_tests:BOOL=OFF \
	-DBUILD_apps:BOOL=ON \
	-DBUILD_gpu_surface:BOOL=ON \
	-DCUDA_NVCC_FLAGS:="--compiler-bindir=${CUDA_BINDIR};-w" \
	-DCMAKE_CXX_FLAGS:=" -Wno-invalid-offsetof -Wno-deprecated-declarations" \
	-DCMAKE_BUILD_TYPE:="RelWithDebInfo" \
	-DBUILD_examples:BOOL=OFF \
	-DCMAKE_INSTALL_PREFIX:=`rospack find pcl_trunk`/debug
	cd svn/build_debug && make -j4 && make install
	rm -rf ${CUDA_BINDIR}
	touch $@


built_against_ros_debug: svn svn/PATCHED_KINFU
	cd svn && mkdir -p build_against_ros_debug
	mkdir -p ${CUDA_BINDIR}
	cd ${CUDA_BINDIR} && ln -sf /usr/bin/gcc-4.4 gcc && ln -sf /usr/bin/g++-4.4 g++
	cd svn/build_against_ros_debug && cmake ../ \
	-DCMAKE_C_COMPILER:FILEPATH=${C_COMPILER} \
	-DCMAKE_CXX_COMPILER:FILEPATH=${CXX_COMPILER} \
	-DUSE_ROS:BOOL=ON \
	-DBUILD_CUDA:BOOL=ON \
	-DBUILD_GPU:BOOL=ON \
	-DBUILD_global_tests:BOOL=OFF \
	-DBUILD_apps:BOOL=ON \
	-DBUILD_gpu_surface:BOOL=ON \
	-DCUDA_NVCC_FLAGS:="--compiler-bindir=${CUDA_BINDIR};-w" \
	-DCMAKE_CXX_FLAGS:=" -Wno-invalid-offsetof -Wno-deprecated-declarations -I`rospack find sensor_msgs`/include -I`rospack find sensor_msgs`/msg_gen/cpp/include -I`rospack find std_msgs`/include -I`rospack find std_msgs`/msg_gen/cpp/include" \
	-DCMAKE_BUILD_TYPE:="RelWithDebInfo" \
	-DBUILD_examples:BOOL=OFF \
	-DCMAKE_INSTALL_PREFIX:=`rospack find pcl_trunk`/debug_against_ros
	cd svn/build_against_ros_debug && make -j4 && make install
	rm -rf ${CUDA_BINDIR}
	touch $@

svn:
	#svn co -r${PCL_VERSION} http://svn.pointclouds.org/pcl/trunk $@
	svn co -r${PCL_VERSION} svn+ssh://svn@svn.pointclouds.org/pcl/trunk $@
	touch svn/rospack_nosubdirs

svn/PATCHED_KINFU:
	cd svn && patch -p0 < ../kinfu_reset.patch
	touch $@

svn/PATCHED_USE_ROS:
	cd svn && patch -p0 < ../use_ros.patch
	touch $@

remove_msgs:
	cd svn/common/include; \
	if [ -d std_msgs ]; then \
	mv std_msgs std_msgs_pcl_do_not_use; \
	mv sensor_msgs sensor_msgs_pcl_do_not_use; \
	fi

remove_ros_conflicts:
	cd svn/common/include; \
	mkdir -p pcl_ros_conflicts; \
	for f in pcl/ModelCoefficients.h pcl/PointIndices.h pcl/PolygonMesh.h pcl/Vertices.h; do \
	if [ -e $${f} ]; then \
	mv $${f} pcl_ros_conflicts; \
	fi; \
	done

restore_ros_conflicts:
	cd svn/common/include; \
	mv pcl_ros_conflicts/* pcl; \
	rm -rf pcl_ros_conflicts

restore_msgs:
	cd svn/common/include; \
	if [ -d std_msgs_pcl_do_not_use ]; then \
	mv std_msgs_pcl_do_not_use std_msgs; \
	mv sensor_msgs_pcl_do_not_use sensor_msgs; \
	fi

clean: restore_msgs restore_ros_conflicts
	cd svn/build && make clean
	-rm -f built

wipe:
	-rm -rf svn msgs_tmp
	-rm -f built

3rdparty: svn
	mkdir 3rdparty
	cd 3rdparty && wget http://people.cs.ubc.ca/~mariusm/uploads/FLANN/flann-1.7.1-src.zip
	cd 3rdparty && unzip flann-1.7.1-src.zip
	mkdir -p flann-1.7.1-src/build
	cd 3rdparty/flann-1.7.1-src/build && cmake ../ -DBUILD_CUDA_LIB:BOOL=OFF -DCMAKE_INSTALL_PREFIX:STRING=/usr
	cd 3rdparty/flann-1.7.1-src/build && make
	cd 3rdparty/flann-1.7.1-src/build && sudo make install


