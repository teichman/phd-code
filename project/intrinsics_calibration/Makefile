
############################################################
# -- Global settings.
############################################################

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.

export NUM_THREADS	:= 40
#SHELL = $(warning ---------- Making $@$(if $?,. Newer prereqs: $?)) /bin/bash -x
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

# These all sucked.
# 2012-12-19_short_00 \
# 2012-12-19_short_02 \
# 2012-12-19_short_06 \
# 2012-12-19_short_07 \
# 2012-12-19_short_08 \
# 2012-12-19_short_09 \
# 2012-12-19_short_10 \
# 2012-12-19_short_11
SLAM_RUNS_TRAINING_xpl00	:=	2012-12-19_boardroom_0 \
					2012-12-19_boardroom_1 \
					2012-12-19_short_05 \
					2013-01-07_2a_backandforth_01 \
					2013-01-07_boardroom_01

SLAM_RUNS_TESTING_xpl00		:=	2012-12-19_short_01 \
					2012-12-19_short_03 \
					2012-12-19_short_04 \
					2012-12-19_boardroom_2 \
					2013-01-07_2a_backandforth_02 \
					2013-01-07_boardroom_02

SLAM_RUNS_xpl00 := $(SLAM_RUNS_TRAINING_xpl00) $(SLAM_RUNS_TESTING_xpl00)

SLAM_RUNS_TRAINING_xpl01	:= 	2a_backandforth \
					2012-10-22_2a_backandforth_0 \
					2012-10-22_2a_backandforth_2 \
					2012-10-22_2a_backandforth_4 \
					2012-11-16_2a_backandforth_6
SLAM_RUNS_TESTING_xpl01		:=	2012-11-16_2a_backandforth_7 \
					2012-11-16_2a_backandforth_8

#SLAM_RUNS_TRAINING_xpl01	:= 	2012-10-22_2a_backandforth_2
#SLAM_RUNS_TESTING_xpl01		:=	2012-11-16_2a_backandforth_7

SLAM_RUNS_xpl01			:= 	$(SLAM_RUNS_TRAINING_xpl01) $(SLAM_RUNS_TESTING_xpl01)


# Data specifically for the 2012-11-06 run in which multiple sensor views were collected.
AVV_MULTIVIEW_NAMES_xpl01	:= 	xpl01_straight-11-06-2012_00-08-08
#					xpl01_down-11-06-2012_00-17-19
#					xpl01_up-11-06-2012_00-12-51
AVV_MULTIVIEW_PATHS_xpl01	:=	$(addprefix $(AVV_PATH)/2012-11-06_primesense_vs_velo/, $(AVV_MULTIVIEW_NAMES_xpl01))

AVV_MULTIVIEW_PATHS_xpl04	:=	$(AVV_PATH)/2012-11-06_primesense_vs_velo/xpl04_down-11-05-2012_23-47-23 \
					$(AVV_PATH)/2012-11-06_primesense_vs_velo/xpl04_straight-11-06-2012_00-01-38 \
					$(AVV_PATH)/2012-11-06_primesense_vs_velo/xpl04_up-11-05-2012_23-53-26

all:
	@echo You should probably be more specific.

############################################################
# RSS 2013 paper
############################################################

rss2013.pdf: 	$$(MAKE_ROOT)/rss2013/rss2013.tex
	cd $(MAKE_ROOT)/rss2013 && pdflatex rss2013
	cd $(MAKE_ROOT)/rss2013 && bibtex rss2013
	cd $(MAKE_ROOT)/rss2013 && pdflatex rss2013
	cd $(MAKE_ROOT)/rss2013 && pdflatex rss2013
	-cd $(MAKE_ROOT)/rss2013 && rm rss2013.{aux,bbl,blg,log,idx} latex_macros.aux
	mv $(MAKE_ROOT)/rss2013/rss2013.pdf .


############################################################
# Experiments
############################################################

%-experiment_metadata.txt:
	mkdir -p $(@D)
	echo `hostname` > $@
	echo `date +%F_%T` >> $@
	echo '' >> $@
	echo '' >> $@
	env >> $@
	echo '' >> $@
	echo '' >> $@
	git log -n1 >> $@
	git diff >> $@


# -- Enumeration of targets for each sensor.
define SENSOR_RULE_GENERATOR

# Now "trajectories" is a directory which contains all submaps and the posegraph
TRAJDIRS_$(1) := $$(addprefix $(1)/slam_results/, $$(SLAM_RUNS_$(1)))
TRAJDIRS_TRAINING_$(1) := $$(addprefix $(1)/slam_results/, $$(SLAM_RUNS_TRAINING_$(1)))
TRAJDIRS_TESTING_$(1) := $$(addprefix $(1)/slam_results/, $$(SLAM_RUNS_TESTING_$(1)))
TRAJECTORIES_ONLYFIRST_$(1) := `for DIR in $$(TRAJDIRS_$(1)); do find $$$${DIR} -name *.traj | sort | head -n1; done`
TRAJECTORIES_ONLYFIRST_TRAINING_$(1) := `for DIR in $$(TRAJDIRS_TRAINING_$(1)); do find $$$${DIR} -name *.traj | sort | head -n1; done`
TRAJECTORIES_ONLYFIRST_TESTING_$(1) := `for DIR in $$(TRAJDIRS_TESTING_$(1)); do find $$$${DIR} -name *.traj | sort | head -n1; done`

TRAJECTORIES_SUB_$(1) := `for DIR in $$(TRAJDIRS_$(1)); do find $$$${DIR} -name *.traj; done`
SLAM_RUNS_DUPL_$(1) := `for DIR in $$(TRAJDIRS_$(1)); do \
	for TRAJ in $$$$(ls $$$${DIR} | grep traj); do basename $$$${DIR} | xargs -I {} echo $(SLAM_DATA_PATH)/$(1)/{}; done; done`

$(1)/evaluation/iterative_calibration_experiment: $$(TRAJDIRS_$(1)) $$$$@-experiment_metadata.txt
	mkdir -p $$(@D)
	rosrun xpl_calibration iterative_calibration_experiment \
	--output $$@ \
	--sseqs-train $$(addprefix $(SLAM_DATA_PATH)/$(1)/, $$(SLAM_RUNS_TRAINING_$(1))) \
	--trajs-train $$(TRAJECTORIES_ONLYFIRST_TRAINING_$(1)) \
	--sseqs-test $$(addprefix $(SLAM_DATA_PATH)/$(1)/, $$(SLAM_RUNS_TESTING_$(1))) \
	--trajs-test $$(TRAJECTORIES_ONLYFIRST_TESTING_$(1))

$(1)/evaluation/slam_eval: $$(TRAJDIRS_$(1)) $$$$@-experiment_metadata.txt
	mkdir -p $$(@D)
	rosrun xpl_calibration evaluate_slam_calibration \
	--output $(1)/evaluation/slam_eval \
	--sseqs $$(addprefix $(SLAM_DATA_PATH)/$(1)/, $$(SLAM_RUNS_$(1))) \
	--trajs $$(TRAJECTORIES_ONLYFIRST_$(1))

$(1)/evaluation/slam_eval_quantity/quantity_experiment.pdf: $(MAKE_ROOT)/plot_quantity_experiment.py $(1)/evaluation/slam_eval_quantity
	python $$^

$(1)/evaluation/slam_eval_quantity: $$(TRAJDIRS_$(1)) $$$$@-experiment_metadata.txt
	mkdir -p $$(@D)
	rosrun xpl_calibration evaluate_quantity \
	--output $$@ \
	--sseqs-train $$(addprefix $(SLAM_DATA_PATH)/$(1)/, $$(SLAM_RUNS_TRAINING_$(1))) \
	--trajs-train $$(TRAJECTORIES_ONLYFIRST_TRAINING_$(1)) \
	--sseqs-test $$(addprefix $(SLAM_DATA_PATH)/$(1)/, $$(SLAM_RUNS_TESTING_$(1))) \
	--trajs-test $$(TRAJECTORIES_ONLYFIRST_TESTING_$(1))

$(1)/evaluation/focal_length: $$(TRAJDIRS_$(1)) $$$$@-experiment_metadata.txt
	mkdir -p $$(@D)
	rosrun xpl_calibration focal_length_experiment \
	--initial-f 550 \
	--output $$@ \
	--sseqs-train $$(addprefix $(SLAM_DATA_PATH)/$(1)/, $$(SLAM_RUNS_TRAINING_$(1))) \
	--trajs-train $$(TRAJECTORIES_ONLYFIRST_TRAINING_$(1)) \
	--sseqs-test $$(addprefix $(SLAM_DATA_PATH)/$(1)/, $$(SLAM_RUNS_TESTING_$(1))) \
	--trajs-test $$(TRAJECTORIES_ONLYFIRST_TESTING_$(1))

$(1)-quantitative-evaluation: 	$(1)/evaluation/results-baseline.txt \
				$(1)/evaluation/results-distortion_model_and_recalib.txt
	@grep '^' $$^

$(1)/evaluation/results-distortion_model_and_recalib.txt: $(1)/evaluation/learned_model.psm $(1)/evaluation/extrinsics 
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--intrinsics $$< --extrinsics $$(@D)/extrinsics \
	--evaluate $$@

# Sanity check.
$(1)/evaluation/results-distortion_model_no_recalib.txt: $(1)/evaluation/learned_model.psm
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--intrinsics $$< --extrinsics $(1)/asus_vs_velodyne/extrinsics \
	--evaluate $$@

$(1)/evaluation/results-baseline.txt: $(1)/asus_vs_velodyne/extrinsics
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--extrinsics $(1)/asus_vs_velodyne/extrinsics \
	--evaluate $$@

$(1)-visualize-slam-intrinsics: $(1)/evaluation/extrinsics $(1)/evaluation/learned_model.psm
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--extrinsics $(1)/evaluation/extrinsics \
	--intrinsics $(1)/evaluation/learned_model.psm

# Visualize intrinsics with the default extrinsics
#$(1)-visualize-slam-discrete-intrinsics: $(1)/evaluation/discrete_learned_model $(1)/asus_vs_velodyne/extrinsics # $(1)/evaluation/discrete_extrinsics
#	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--extrinsics $(1)/asus_vs_velodyne/extrinsics \
	--discrete-intrinsics $(1)/evaluation/discrete_learned_model

# Visualize intrinsics with the extrinsics learned specifically for this set of intrinsics.
$(1)-visualize-slam-discrete-intrinsics: $(1)/evaluation/discrete_learned_model $(1)/evaluation/discrete_extrinsics
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--discrete-intrinsics $(1)/evaluation/discrete_learned_model \
	--extrinsics $(1)/evaluation/discrete_extrinsics

$(1)/evaluation/extrinsics: $(1)/evaluation/learned_model.psm
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--intrinsics learned_model.psm \
	--compute-extrinsics $$@

$(1)/evaluation/discrete_extrinsics: $(1)/evaluation/discrete_learned_model
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--discrete-intrinsics $$< \
	--compute-extrinsics $$@;

$(1)/evaluation/learned_model.psm: $$(TRAJDIRS_$(1)) $$$$@-experiment_metadata.txt
	mkdir -p $$(@D)
	rosrun xpl_calibration calibrate_slam \
	$$$$(for f in $$(SLAM_RUNS_DUPL_$(1)); do echo --sseq $$$$f; done) \
	$$$$(for f in $$(TRAJECTORIES_SUB_$(1)); do echo --traj $$$$f; done) \
	--omodel $$@ \
	| tee $$@-log.txt
	rm -rf $$(@D)/clams_coverage
	mv .ddl $$(@D)/clams_coverage

$(1)/evaluation/discrete_learned_model: $$(TRAJDIRS_$(1)) $$$$@-experiment_metadata.txt
	mkdir -p $$(@D)
	rosrun xpl_calibration calibrate_slam \
	--discrete \
	$$$$(for f in $$(SLAM_RUNS_DUPL_$(1)); do echo --sseq $$$$f; done) \
	$$$$(for f in $$(TRAJECTORIES_SUB_$(1)); do echo --traj $$$$f; done) \
	--omodel $$@ \
	| tee $$@-log.txt
	rm -rf $$(@D)/clams_coverage
	mv .ddl $$(@D)/clams_coverage

$(1)-visualize-avv-intrinsics: $(1)/asus_vs_velodyne/extrinsics $(1)/asus_vs_velodyne/intrinsics
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence $$(AVV_PATH_$(1))-VeloSequence \
	--extrinsics $(1)/asus_vs_velodyne/extrinsics \
	--intrinsics $(1)/asus_vs_velodyne/intrinsics

$(1)/asus_vs_velodyne/intrinsics: $(1)/asus_vs_velodyne/extrinsics
	mkdir -p $$(@D)
	cd $$(@D) && \
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence \
	$$(AVV_PATH_$(1))-VeloSequence \
	--extrinsics extrinsics \
	--compute-intrinsics $$@

$(1)-visualize-avv-extrinsics: $(1)/asus_vs_velodyne/extrinsics
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence \
	$$(AVV_PATH_$(1))-VeloSequence \
	--extrinsics $$<

$(1)/asus_vs_velodyne/extrinsics:
	mkdir -p $$(@D)
	cd $$(@D) && \
	rosrun xpl_calibration asus_vs_velo \
	$$(AVV_PATH_$(1))-StreamSequence \
	$$(AVV_PATH_$(1))-VeloSequence \
	--compute-extrinsics $$@

$(1)-visualize-slam: $$(TRAJDIRS_$(1))
	rosrun xpl_calibration calibrate_slam --only-visualize \
	$$$$(for f in $$(SLAM_RUNS_DUPL_$(1)); do echo --sseq $$$$f; done) \
	$$$$(for f in $$(TRAJECTORIES_SUB_$(1)); do echo --traj $$$$f; done)

.PHONY: $(1)/slam_results
$(1)/slam_results: $$(TRAJDIRS_$(1))

.PHONY: $(1)/slam_videos
$(1)/slam_videos: $$(addprefix $(1)/slam_videos/, $$(addsuffix .avi, $$(SLAM_RUNS_$(1))))


endef
$(foreach SENSOR,$(SENSORS),$(eval $(call SENSOR_RULE_GENERATOR,$(SENSOR))))


# -- Enumeration of targets for each run of each sensor.
define SENSOR_RUN_RULE_GENERATOR

$(1)-visualize-intrinsics-on-map-$(2): $(1)/evaluation/slam_eval
	cd $(1)/evaluation/slam_eval/$(2) && rosrun xpl_calibration visualize_slam_calibration --imodel intrinsics $(SLAM_DATA_PATH)/$(1)/$(2)/ ../../../slam_results/$(2)/submap_0.traj

# Press j to take a screenshot from the angle you want.
$(1)/user_defined_camera-$(2): $(SLAM_DATA_PATH)/$(1)/$(2) $(CAMERAS_PATH)/$(1)/$(2).cam $(1)/slam_results/$(2)
	#TODO dont only use first submap
	SUBMAP=`find $(1)/slam_results/$(2) -name "*.pcd" | head -1` && rosrun pcl_bleeding pcd_viewer $$$${SUBMAP}
	#rosrun pcl_bleeding pcd_viewer $(1)/slam_results/$(2).pcd -cam $(CAMERAS_PATH)/$(1)/$(2).cam
	mv `ls screenshot*.cam -tr | tail -n1` $(CAMERAS_PATH)/$(1)/$(2).cam
	rm `ls screenshot*.png -tr | tail -n1`

#$(1)-visualize-slam-trajectory-with-learned-model-$(2): $(1)/slam_results/$(2).traj
#	rosrun xpl_calibration calibrate_slam --only-visualize --sseq $(SLAM_DATA_PATH)/$(1)/$(2) --traj $(1)/slam_results/$(2).traj

$(1)-visualize-slam-trajectory-$(2): $(1)/slam_results/$(2)
	for TRAJ in `find $(1)/slam_results/$(2) -name "*.traj"`; do \
	rosrun xpl_calibration calibrate_slam --only-visualize --sseq $(SLAM_DATA_PATH)/$(1)/$(2) --traj $$$${TRAJ}; \
	done

$(1)-visualize-slam-unrefined-trajectory-$(2): $(1)/slam_results/$(2)_unrefined
	for TRAJ in `find $(1)/slam_results/$(2)_unrefined -name "*.traj"`; do \
	rosrun xpl_calibration calibrate_slam --only-visualize --sseq $(SLAM_DATA_PATH)/$(1)/$(2) --traj $$$${TRAJ} ; \
	done

$(1)-visualize-slam-pcd-$(2): $(1)/slam_results/$(2) $(CAMERAS_PATH)/$(1)/$(2).cam
	for PCD in `find $(1)/slam_results/$(2) -name "*.pcd"`; do \
	rosrun pcl_bleeding pcd_viewer $$$${PCD} -cam $(CAMERAS_PATH)/$(1)/$(2).cam; \
	done

$(1)-visualize-slam-unrefined-pcd-$(2): $(1)/slam_results/$(2)_unrefined $(CAMERAS_PATH)/$(1)/$(2).cam
	for PCD in `find $(1)/slam_results/$(2)_unrefined -name "*.pcd"`; do \
	rosrun pcl_bleeding pcd_viewer $$$${PCD} -cam $(CAMERAS_PATH)/$(1)/$(2).cam; \
	done

.PHONY: $(1)-inspect-posegraph-$(2)
$(1)-inspect-posegraph-$(2): $(SLAM_DATA_PATH)/$(1)/$(2) $(1)/slam_results/$(2)_unrefined
	rosrun xpl_calibration inspect_pose_graph $$< $(1)/slam_results/$(2)_unrefined/graph.log --visualize

.PHONY: $(1)-view-links-$(2)
$(1)-view-links-$(2): $(SLAM_DATA_PATH)/$(1)/$(2) $(1)/slam_results/$(2)_unrefined
	rosrun xpl_calibration view_links $$< $(1)/slam_results/$(2)_unrefined/graph.log

$(1)/slam_results/$(2): $(SLAM_DATA_PATH)/$(1)/$(2) $(1)/slam_results/$(2)_unrefined
	rosrun xpl_calibration inspect_pose_graph $$< $(1)/slam_results/$(2)_unrefined/graph.log --otraj $(1)/slam_results/$(2) --opcd $(1)/slam_results/$(2) --ograph $(1)/slam_results/$(2)/graph.log

.PHONY: $(1)-remake-unrefined-$(2)
$(1)-remake-unrefined-$(2): $(SLAM_DATA_PATH)/$(1)/$(2) $(1)/slam_results/$(2)_unrefined
	rm -rf $(1)/slam_results/$(2)_unrefined/submap* $(1)/slam_results/$(2)
	rosrun xpl_calibration inspect_pose_graph $$< $(1)/slam_results/$(2)_unrefined/graph.log --otraj $(1)/slam_results/$(2)_unrefined --opcd $(1)/slam_results/$(2)_unrefined --threshold 9999999


# $(1)/slam_videos/$(2).avi: $(SLAM_DATA_PATH)/$(1)/$(2) $(CAMERAS_PATH)/$(1)/$(2).cam
# 	mkdir -p $$(@D)/$(2)
# 	cd $$(@D)/$(2) && GRIDSEARCH_VIS=1 rosrun xpl_calibration slam_test $$< $(2).pcd $(2).traj --cam $(CAMERAS_PATH)/$(1)/$(2).cam --save-imgs ${LOOP_CLOSURE_FLAG}
# 	cd $$(@D)/$(2) && mkdir slam-png && mv slam*.png slam-png/
# 	cd $$(@D)/$(2) && mkdir montage-png && mv montage*.png montage-png/	
# 	cd $$(@D)/$(2) && mencoder "mf://slam-png/slam*.png" -mf fps=20 -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=10000000 -o $(2)-alignment.avi
# 	cd $$(@D)/$(2) && mencoder "mf://montage-png/montage*.png" -mf fps=20 -ovc lavc -lavcopts vcodec=msmpeg4v2:vbitrate=10000000 -o $(2)-montage.avi

$(1)/slam_results/$(2)_unrefined: $(SLAM_DATA_PATH)/$(1)/$(2) $(CAMERAS_PATH)/$(1)/$(2).cam
	mkdir -p $$(@D)
	rosrun xpl_calibration primesense_slam $$< --otraj $$@ --opcd $$@ --ograph $$@/graph.log --cam $(CAMERAS_PATH)/$(1)/$(2).cam

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


############################################################
# Asus vs Velodyne, multiple views
############################################################

all-multiview-extrinsics: 	xpl01/asus_vs_velodyne-multiview/xpl01_down-11-06-2012_00-17-19/extrinsics \
				xpl01/asus_vs_velodyne-multiview/xpl01_straight-11-06-2012_00-08-08/extrinsics \
				xpl01/asus_vs_velodyne-multiview/xpl01_up-11-06-2012_00-12-51/extrinsics \
				xpl04/asus_vs_velodyne-multiview/xpl04_down-11-05-2012_23-47-23/extrinsics \
				xpl04/asus_vs_velodyne-multiview/xpl04_straight-11-06-2012_00-01-38/extrinsics \
				xpl04/asus_vs_velodyne-multiview/xpl04_up-11-05-2012_23-53-26/extrinsics

define MULTIVIEW_AVV_SENSOR_RULE_GENERATOR

AVV_MULTIVIEW_SSEQ_PATHS_$(1)		:=	$$(addsuffix -StreamSequence, $$(AVV_MULTIVIEW_PATHS_$(1)))
AVV_MULTIVIEW_VSEQ_PATHS_$(1)		:=	$$(addsuffix -VeloSequence, $$(AVV_MULTIVIEW_PATHS_$(1)))
AVV_MULTIVIEW_EXTRINSICS_PATHS_$(1)	:=	$$(addprefix $(1)/asus_vs_velodyne-multiview/, $$(addsuffix /extrinsics, $$(AVV_MULTIVIEW_NAMES_$(1))))

$(1)/asus_vs_velodyne-multiview/intrinsics.psm: $$(AVV_MULTIVIEW_EXTRINSICS_PATHS_$(1)) $$$$@-experiment_metadata.txt
	mkdir -p $$(@D)
	rosrun xpl_calibration avv_multiview_calibrator \
	--sseqs $$(AVV_MULTIVIEW_SSEQ_PATHS_$(1)) \
	--vseqs $$(AVV_MULTIVIEW_VSEQ_PATHS_$(1)) \
	--extrinsics $$(AVV_MULTIVIEW_EXTRINSICS_PATHS_$(1)) \
	--output $$@ | tee $$@-log.txt

$(1)/asus_vs_velodyne-multiview/discrete_distortion_model: $$(AVV_MULTIVIEW_EXTRINSICS_PATHS_$(1)) $$$$@-experiment_metadata.txt
	mkdir -p $$(@D)
	rosrun xpl_calibration avv_multiview_calibrator \
	--sseqs $$(AVV_MULTIVIEW_SSEQ_PATHS_$(1)) \
	--vseqs $$(AVV_MULTIVIEW_VSEQ_PATHS_$(1)) \
	--extrinsics $$(AVV_MULTIVIEW_EXTRINSICS_PATHS_$(1)) \
	--discrete \
	--output $$@ | tee $$@-log.txt

endef
$(foreach SENSOR,xpl01 xpl04,$(eval $(call MULTIVIEW_AVV_SENSOR_RULE_GENERATOR,$(SENSOR))))


MULTIVIEW_PATH := $(AVV_PATH)/2012-11-06_primesense_vs_velo

define MULTIVIEW_AVV_EXTRINSICS_RULE_GENERATOR

$(1)-visualize-multiview-avv-intrinsics-$(2): $(1)/asus_vs_velodyne-multiview/intrinsics.psm $(1)/asus_vs_velodyne-multiview/$(2)/extrinsics
	rosrun xpl_calibration asus_vs_velo \
	$(MULTIVIEW_PATH)/$(2)-StreamSequence \
	$(MULTIVIEW_PATH)/$(2)-VeloSequence \
	--extrinsics $(1)/asus_vs_velodyne-multiview/$(2)/extrinsics \
	--intrinsics $(1)/asus_vs_velodyne-multiview/intrinsics.psm 

$(1)-visualize-multiview-avv-discrete-intrinsics-$(2): $(1)/asus_vs_velodyne-multiview/discrete_distortion_model $(1)/asus_vs_velodyne-multiview/$(2)/extrinsics
	rosrun xpl_calibration asus_vs_velo \
	$(MULTIVIEW_PATH)/$(2)-StreamSequence \
	$(MULTIVIEW_PATH)/$(2)-VeloSequence \
	--extrinsics $(1)/asus_vs_velodyne-multiview/$(2)/extrinsics \
	--discrete-intrinsics $(1)/asus_vs_velodyne-multiview/discrete_distortion_model 

$(1)-visualize-multiview-avv-extrinsics-$(2): $(1)/asus_vs_velodyne-multiview/$(2)/extrinsics
	rosrun xpl_calibration asus_vs_velo \
	$(MULTIVIEW_PATH)/$(2)-StreamSequence \
	$(MULTIVIEW_PATH)/$(2)-VeloSequence \
	--extrinsics $$<

$(1)/asus_vs_velodyne-multiview/$(2)/extrinsics:
	mkdir -p $$(@D)
	cd $$(@D) && \
	rosrun xpl_calibration asus_vs_velo \
	$(MULTIVIEW_PATH)/$(2)-StreamSequence \
	$(MULTIVIEW_PATH)/$(2)-VeloSequence \
	--compute-extrinsics $$@

endef
$(foreach SENSOR,xpl01 xpl04,$(foreach RUN,$(shell ls $(AVV_PATH)/2012-11-06_primesense_vs_velo | grep $(SENSOR) | grep VeloSequence | sed 's/-VeloSequence//g'),$(eval $(call MULTIVIEW_AVV_EXTRINSICS_RULE_GENERATOR,$(SENSOR),$(RUN)))))
