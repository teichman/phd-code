all: teichman-ros-pkg
	rosmake core external

# This Makefile can also be called from teichman_ros_pkg_hook, so we need to set the root dir.
ROOT_DIR := $(shell rospack find dst)/../../
TRP_REV := 720ebc3d4931df25299452c9e8493c9d12ea0d87
#TRP_REV := a32bda52733c4b7a08670c4b15c277da2f03b071
.PHONY: teichman-ros-pkg
teichman-ros-pkg:
	@if [ ! -e $(ROOT_DIR)/.teichman-ros-pkg_rev$(TRP_REV) ]; then \
	echo Updating teichman-ros-pkg...; \
	cd $(ROOT_DIR) && rm -rf $@; \
	cd $(ROOT_DIR) && git clone git://github.com/teichman/teichman-ros-pkg.git; \
	cd $(ROOT_DIR)/$@ && git checkout $(TRP_REV); \
	cd $(ROOT_DIR) && rm -rf $@/stacks/utility/teichman_ros_pkg_hook; \
	cd $(ROOT_DIR) && rm -f .teichman-ros-pkg_rev*; \
	cd $(ROOT_DIR) && touch .teichman-ros-pkg_rev$(TRP_REV); \
	else \
	echo No need to update teichman-ros-pkg, doing nothing.; \
	fi

clean: 
	rosmake --target=clean core external utility machine_learning

test:
	rosmake --target=test core external utility machine_learning

wipe:
	rm -rf teichman-ros-pkg .teichman-ros-pkg_rev*
	rosmake --target=wipe core external

