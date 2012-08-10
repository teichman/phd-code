all: teichman-ros-pkg
	rosmake core external

TRP_REV := 720ebc3d4931df25299452c9e8493c9d12ea0d87
.PHONY: teichman-ros-pkg
teichman-ros-pkg:
	if [ ! -e .teichman-ros-pkg-$TRP_REV ]; do \
	rm -rf $@; \
	git clone git://github.com/teichman/teichman-ros-pkg.git; \
	cd $@ && git checkout $TRP_REV; \
	rm -rf $@/stacks/utility/teichman_ros_pkg_hook; \
	touch .$TRP_REV; \
	fi

clean: 
	rosmake --target=clean core external utility machine_learning

test:
	rosmake --target=test core external utility machine_learning

wipe:
	rm -rf teichman-ros-pkg .teichman-ros-pkg-*
	rosmake --target=wipe core external

