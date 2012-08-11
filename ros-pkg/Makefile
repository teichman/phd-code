

SHELL := /bin/bash
# This Makefile can also be called from teichman_ros_pkg_hook, so we need to set the root dir.
ROOT_DIR := $(shell dirname $$(dirname $$(rospack find dst)))

# Desired revision is found here.
include $(ROOT_DIR)/external/teichman_ros_pkg_hook-disabled/Makefile

TRP_FLAG := .teichman-ros-pkg-$(TRP_REV)

#all: teichman-ros-pkg
#	rosmake utility external core

.PHONY: teichman-ros-pkg
teichman-ros-pkg:
	@if [ ! -e $(ROOT_DIR)/$(TRP_FLAG) ]; then \
	echo ========================================; \
	echo Updating teichman-ros-pkg...; \
	echo ========================================; \
	cd $(ROOT_DIR) && rm -rf teichman-ros-pkg; \
	cd $(ROOT_DIR) && git clone git://github.com/teichman/teichman-ros-pkg.git; \
	cd $(ROOT_DIR)/teichman-ros-pkg && git checkout $(TRP_REV); \
	cd $(ROOT_DIR) && rm -f .teichman-ros-pkg-*; \
	cd $(ROOT_DIR) && touch $(TRP_FLAG); \
	rosmake utility; \
	else \
	echo ========================================; \
	echo teichman-ros-pkg is up to date already; \
	echo ========================================; \
	fi
#	cd $(ROOT_DIR) && rm -rf teichman-ros-pkg/stacks/utility/teichman_ros_pkg_hook; \

clean: 
	rosmake --target=clean core external utility machine_learning

test:
	rosmake --target=test core external utility machine_learning

wipe:
	rm -rf teichman-ros-pkg .teichman-ros-pkg-*
	rosmake --target=wipe core external

