
############################################################
# -- Global settings.
############################################################
export NUM_THREADS	:= 40
SHELL			:= /bin/bash
REPO_ROOT		:= $(shell dirname $$(dirname $$(dirname $$(rospack find dst))))
MAKE_ROOT		:= $(REPO_ROOT)/project/intrinsics_calibration
MAKEFILE		:= $(MAKE_ROOT)/Makefile

# You'll need a symlink in your project dir to the data.
# This dir has subdirectories of asus_vs_velodyne
DATA_PATH		:= $(MAKE_ROOT)/data
AVV_PATH		:= $(DATA_PATH)/asus_vs_velodyne
SLAM_DATA_PATH		:= $(DATA_PATH)/slam_sequences
CAMERAS_PATH		:= $(MAKE_ROOT)/camera_angles

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.

all:
	@echo You should probably be more specific.


############################################################
# SLAM
############################################################

SLAM_RUNS := $(shell ls $(SLAM_DATA_PATH))

.PHONY: slam_videos
slam_videos: $(addprefix slam_videos/, $(addsuffix .avi, $(SLAM_RUNS)))
.PHONY: slam_results
slam_results: $(addprefix slam_results/, $(addsuffix .pcd, $(SLAM_RUNS)))

# -- Autocompletion is easier with this syntax.
define SLAM_RULE_GENERATOR

view_slam_result-$(1): slam_results/$(1).pcd
	rosrun pcl_bleeding pcd_viewer slam_results/$(1).pcd -cam $(CAMERAS_PATH)/$(1).cam

slam_videos/$(1).avi: $(SLAM_DATA_PATH)/$(1) $(CAMERAS_PATH)/$(1).cam
	mkdir -p $$(@D)/$(1)
	cd $$(@D)/$(1) && rosrun xpl_calibration slam_test $$< $(1).pcd --cam $(CAMERAS_PATH)/$(1).cam --save-imgs
	cd $$(@D)/$(1) && mkdir png && mv *.png png/
	mencoder "mf://$$(@D)/$(1)/png/slam*.png" -mf fps=20 -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=10000000 -o $$@

slam_results/$(1).pcd: $(SLAM_DATA_PATH)/$(1) $(CAMERAS_PATH)/$(1).cam
	mkdir -p $$(@D)
	rosrun xpl_calibration slam_test $$< $$@ --cam $(CAMERAS_PATH)/$(1).cam

$(CAMERAS_PATH)/$(1).cam:
	cp $(CAMERAS_PATH)/default.cam $$@

endef
$(foreach VAL,$(SLAM_RUNS),$(eval $(call SLAM_RULE_GENERATOR,$(VAL))))

#visualize_slam-%: slam_sequences/%/trajectory_estimate.txt
#	rosrun slam_interface view_reconstruction $(SLAM_DATA_PATH)/$*/ slam_sequences/$*/$*_RGBDSLAM/trajectory_estimate.txt

#slam_trajectories: $(addsuffix /trajectory_estimate.txt, $(addprefix slam_sequences/, $(SLAM_RUNS)))
#slam_sequences/%/trajectory_estimate.txt:
#	mkdir -p $(@D)
#	roslaunch slam_interface sseq_slam.launch sseq:=$(SLAM_DATA_PATH)/$*
#	mv $(SLAM_DATA_PATH)/$*_RGBDSLAM $(@D)
#	cp $(@D)/$*_RGBDSLAM/trajectory_estimate.txt $@

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


visualize_avv-%: asus_vs_velodyne/%/extrinsics
	rosrun xpl_calibration asus_vs_velo $(AVV_PATH)/$*/asus $(AVV_PATH)/$*/velodyne --extrinsics $<