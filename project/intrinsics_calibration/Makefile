
############################################################
# -- Global settings.
############################################################

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.

export NUM_THREADS	:= 40
SHELL			:= /bin/bash
REPO_ROOT		:= $(shell dirname $$(dirname $$(dirname $$(rospack find dst))))
MAKE_ROOT		:= $(REPO_ROOT)/project/intrinsics_calibration
MAKEFILE		:= $(MAKE_ROOT)/Makefile

# You'll need a symlink in your project dir to the data.
# This dir has subdirectories of asus_vs_velodyne
DATA_PATH		:= $(MAKE_ROOT)/data

SLAM_DATA_PATH		:= $(DATA_PATH)/slam_sequences
CAMERAS_PATH		:= $(MAKE_ROOT)/camera_angles
SENSORS			:= $(shell ls $(SLAM_DATA_PATH))
AVV_PATH		:= $(DATA_PATH)/asus_vs_velodyne
AVV_PATH_xpl01		:= $(AVV_PATH)/2012-09-13_primesense_vs_velo/intrinsics_xpl_01_taketwo-09-13-2012_00-09-54
AVV_PATH_xpl04		:= $(AVV_PATH)/2012-09-13_primesense_vs_velo/intrinsics_xpl_04-09-12-2012_23-35-53
AVV_PATH_kinect00	:= $(AVV_PATH)/2012-09-13_primesense_vs_velo/intrinsics_kinect_00-09-12-2012_23-42-23
AVV_PATH_kinect01	:= $(AVV_PATH)/2012-09-13_primesense_vs_velo/intrinsics_kinect_01-09-13-2012_00-01-35

SLAM_RUNS_xpl01 :=	xpl001-seq004 \
			xpl001-seq005

# Convenient to turn on if you want to watch it run, but not save images
DEFAULT_GRID_VIS := 0
# Turn off loop closure with this
LOOP_CLOSURE_FLAG := --loop-closure

all:
	@echo You should probably be more specific.


############################################################
# Experiments
############################################################


# -- Enumeration of targets for each sensor.
define SENSOR_RULE_GENERATOR

$(1)-visualize-slam-intrinsics: $(1)/asus_vs_velodyne/extrinsics $(1)/learned_model.psm
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--extrinsics $(1)/asus_vs_velodyne/extrinsics \
	--intrinsics $(1)/learned_model.psm

$(1)-visualize-avv-intrinsics: $(1)/asus_vs_velodyne/extrinsics $(1)/asus_vs_velodyne/intrinsics
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--extrinsics $(1)/asus_vs_velodyne/extrinsics \
	--intrinsics $(1)/asus_vs_velodyne/intrinsics

$(1)/asus_vs_velodyne/intrinsics: $(1)/asus_vs_velodyne/extrinsics
	mkdir -p $$(@D)
	cd $$(@D) && rosrun xpl_calibration asus_vs_velo $$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence --extrinsics extrinsics --compute-intrinsics

$(1)-visualize-avv-extrinsics: $(1)/asus_vs_velodyne/extrinsics
	rosrun xpl_calibration asus_vs_velo $$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence --extrinsics $$<

$(1)/asus_vs_velodyne/extrinsics:
	mkdir -p $$(@D)
	cd $$(@D) && rosrun xpl_calibration asus_vs_velo $$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence --compute-extrinsics
	cd $$(@D) && mkdir gridsearch
	cd $$(@D) && mv gridsearch*.png gridsearch/

#SLAM_RUNS_$(1) := $$(shell ls $(SLAM_DATA_PATH)/$(1))
TRAJECTORIES_$(1) := $$(addprefix $(1)/slam_results/, $$(addsuffix .traj, $$(SLAM_RUNS_$(1))))

$(1)/learned_model.psm: $$(TRAJECTORIES_$(1))
	rosrun xpl_calibration calibrate_slam \
	--batch --omodel $$@ \
	$$(addprefix --sseq $(SLAM_DATA_PATH)/$(1)/, $$(SLAM_RUNS_$(1))) \
	$$(addprefix --traj ,$$(TRAJECTORIES_$(1)))

$(1)-visualize-slam: $$(TRAJECTORIES_$(1))
	rosrun xpl_calibration calibrate_slam $$(addprefix --sseq $(SLAM_DATA_PATH)/$(1)/, $$(SLAM_RUNS_$(1))) $$(addprefix --traj ,$$(TRAJECTORIES_$(1)))

.PHONY: $(1)/slam_results
$(1)/slam_results: $$(TRAJECTORIES_$(1))

.PHONY: $(1)/slam_videos
$(1)/slam_videos: $$(addprefix $(1)/slam_videos/, $$(addsuffix .avi, $$(SLAM_RUNS_$(1))))

endef
$(foreach SENSOR,$(SENSORS),$(eval $(call SENSOR_RULE_GENERATOR,$(SENSOR))))


# -- Enumeration of targets for each run of each sensor.
define SENSOR_RUN_RULE_GENERATOR

# Press j to take a screenshot from the angle you want.
$(1)/user_defined_camera-$(2): $(SLAM_DATA_PATH)/$(1)/$(2) $(CAMERAS_PATH)/$(1)/$(2).cam $(1)/slam_results/$(2).traj
	rosrun pcl_bleeding pcd_viewer $(1)/slam_results/$(2).pcd -cam $(CAMERAS_PATH)/$(1)/$(2).cam
	mv `ls screenshot*.cam -tr | tail -n1` $(CAMERAS_PATH)/$(1)/$(2).cam
	rm `ls screenshot*.png -tr | tail -n1`

$(1)-visualize-slam-result-$(2): $(1)/slam_results/$(2).traj $(CAMERAS_PATH)/$(1)/$(2).cam
	rosrun pcl_bleeding pcd_viewer $(1)/slam_results/$(2).pcd -cam $(CAMERAS_PATH)/$(1)/$(2).cam

$(1)/slam_videos/$(2).avi: $(SLAM_DATA_PATH)/$(1)/$(2) $(CAMERAS_PATH)/$(1)/$(2).cam
	mkdir -p $$(@D)/$(2)
	cd $$(@D)/$(2) && GRIDSEARCH_VIS=1 rosrun xpl_calibration slam_test $$< $(2).pcd $(2).traj --cam $(CAMERAS_PATH)/$(1)/$(2).cam --save-imgs ${LOOP_CLOSURE_FLAG}
	cd $$(@D)/$(2) && mkdir slam-png && mv slam*.png slam-png/
	cd $$(@D)/$(2) && mkdir montage-png && mv montage*.png montage-png/	
	cd $$(@D)/$(2) && mencoder "mf://slam-png/slam*.png" -mf fps=20 -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=10000000 -o $(2)-alignment.avi
	cd $$(@D)/$(2) && mencoder "mf://montage-png/montage*.png" -mf fps=20 -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=10000000 -o $(2)-montage.avi

$(1)/slam_results/$(2).traj: $(SLAM_DATA_PATH)/$(1)/$(2) $(CAMERAS_PATH)/$(1)/$(2).cam
	mkdir -p $$(@D)
	GRIDSEARCH_VIS=${DEFAULT_GRID_VIS} rosrun xpl_calibration slam_test $$< $$(@D)/$(2).pcd $$@ --cam $(CAMERAS_PATH)/$(1)/$(2).cam ${LOOP_CLOSURE_FLAG}

$(CAMERAS_PATH)/$(1)/$(2).cam:
	mkdir -p $$(@D)
	cp $(CAMERAS_PATH)/default.cam $$@

endef
$(foreach SENSOR,$(SENSORS),$(foreach RUN,$(shell ls $(SLAM_DATA_PATH)/$(SENSOR)),$(eval $(call SENSOR_RUN_RULE_GENERATOR,$(SENSOR),$(RUN)))))


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


visualize_avv-%: asus_vs_velodyne/%/extrinsics
	rosrun xpl_calibration asus_vs_velo $(AVV_PATH)/$*/asus $(AVV_PATH)/$*/velodyne --extrinsics $<
