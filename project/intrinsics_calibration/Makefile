
############################################################
# -- Global settings.
############################################################
export NUM_THREADS	:= 8
SHELL			:= /bin/bash
REPO_ROOT		:= $(shell dirname $$(dirname $$(dirname $$(rospack find dst))))
MAKE_ROOT		:= $(REPO_ROOT)/project/intrinsics_calibration
MAKEFILE		:= $(MAKE_ROOT)/Makefile

# You'll need a symlink in your project dir to the data.
# This dir has subdirectories of asus_vs_velodyne
DATA_PATH		:= $(MAKE_ROOT)/data
AVV_PATH		:= $(DATA_PATH)/asus_vs_velodyne
SLAM_PATH		:= $(DATA_PATH)/slam_sequences

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.

all:
	@echo You should probably be more specific.


############################################################
# SLAM
############################################################

SLAM_RUNS := $(shell ls $(SLAM_PATH))
slam_trajectories: $(addsuffix /trajectory_estimate.txt, $(addprefix slam_sequences/, $(SLAM_RUNS)))
slam_sequences/%/trajectory_estimate.txt:
	mkdir -p $(@D)
	roslaunch slam_interface sseq_slam.launch sseq:=$(SLAM_PATH)/$*
	mv $(SLAM_PATH)/$*_RGBDSLAM $(@D)
	cp $(@D)/$*_RGBDSLAM/trajectory_estimate.txt $@

############################################################
# Asus vs Velodyne
############################################################

AVV_RUNS := $(shell ls $(AVV_PATH))

avv_imagery: $(addsuffix /imagery, $(addprefix asus_vs_velodyne/, $(AVV_RUNS)))
avv_extrinsics: $(addsuffix /extrinsics, $(addprefix asus_vs_velodyne/, $(AVV_RUNS)))

asus_vs_velodyne/%/imagery: asus_vs_velodyne/%/extrinsics
	mkdir -p $@
	cd $(@D) && rosrun xpl_calibration asus_vs_velo $(AVV_PATH)/$*/asus $(AVV_PATH)/$*/velodyne --extrinsics extrinsics --visualize-distortion --skip=1 --num-pixel-plots 100
	cd $(@D) && mv *.png *.pdf imagery/

asus_vs_velodyne/%/extrinsics:
	mkdir -p $(@D)
	cd $(@D) && rosrun xpl_calibration asus_vs_velo $(AVV_PATH)/$*/asus $(AVV_PATH)/$*/velodyne --compute-extrinsics
	mkdir gridsearch
	mv gridsearch*.png gridsearch/


visualize_avv_%:
	rosrun xpl_calibration asus_vs_velo $(AVV_PATH)/$*/asus $(AVV_PATH)/$*/velodyne