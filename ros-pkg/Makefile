SHELL := /bin/bash
# This Makefile can also be called from teichman_ros_pkg_hook, so we need to set the root dir.
ROOT_DIR := $(shell dirname $$(dirname $$(rospack find dst)))
TRP_REV := 720ebc3d4931df25299452c9e8493c9d12ea0d87  # newer
#TRP_REV := a32bda52733c4b7a08670c4b15c277da2f03b071  # older

#all: .teichman-ros-pkg-$(TRP_REV)
#	rosmake utility external core

.PHONY: teichman-ros-pkg
teichman-ros-pkg: .teichman-ros-pkg-$(TRP_REV)

.teichman-ros-pkg-$(TRP_REV):
	@echo ========================================
	@echo Updating teichman-ros-pkg...
	@echo ========================================
	cd $(ROOT_DIR) && rm -rf teichman-ros-pkg
	cd $(ROOT_DIR) && git clone git://github.com/teichman/teichman-ros-pkg.git
	cd $(ROOT_DIR)/teichman-ros-pkg && git checkout $(TRP_REV)
	cd $(ROOT_DIR) && rm -rf teichman-ros-pkg/stacks/utility/teichman_ros_pkg_hook
	cd $(ROOT_DIR) && rm -f .teichman-ros-pkg-*
	cd $(ROOT_DIR) && touch $@
#	sleep 1
#	rosmake utility  # In theory this should not be necessary...

clean: 
	rosmake --target=clean core external utility machine_learning

test:
	rosmake --target=test core external utility machine_learning

wipe:
	rm -rf teichman-ros-pkg .teichman-ros-pkg-*
	rosmake --target=wipe core external

