
############################################################
# -- Global settings.
############################################################
export NUM_THREADS	:= 8
SHELL			:= /bin/bash
REPO_ROOT		:= $(shell rospack find xpl_calibration)/../..
MAKE_ROOT		:= $(REPO_ROOT)/project/calibration
MAKEFILE		:= $(MAKE_ROOT)/Makefile

# You'll need a symlink in your project dir to the data.
# This dir has subdirectories of configuration0, configuration1, etc.
DATA_DIR		:= $(MAKE_ROOT)/data

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.

all:
	@echo You should probably be more specific.

# -s is useless.
code:
	rosmake xpl_calibration
	rosmake cloud_calibration

CONFIGURATIONS := $(shell ls `rospack find xpl_calibration`/../../project/calibration/data)

CALIBRATE_DYNAMIC := DEBUG=1 ON=30 OFF=50 rosrun xpl_calibration calibrate_sequences --dynamic
CALIBRATE_ORB := rosrun xpl_calibration calibrate_sequences --orb
VISUALIZE := rosrun xpl_calibration calibration_viewer
EVALUATE := rosrun xpl_calibration evaluate

############################################################
# -- Experiments
############################################################

%-experiment_metadata.txt:
	@mkdir -p $(@D)
	@echo `hostname` > $@
	@echo `date +%F_%T` >> $@
	@echo '' >> $@
	@git log | head -n7 >> $@
	@echo '' >> $@
	@env >> $@

all_dynamic: $(addprefix dynamic/, $(addsuffix /transform.eig.txt, $(CONFIGURATIONS)))
all_keypoint: $(addprefix keypoint/, $(addsuffix /transform.eig.txt, $(CONFIGURATIONS)))
all_ground_truth: $(addprefix ground_truth/, $(addsuffix /transform.eig.txt, $(CONFIGURATIONS)))
all_ground_truth_user: $(addprefix ground_truth_user/, $(addsuffix /transform.eig.txt, $(CONFIGURATIONS)))

visualize_dynamic-%: dynamic/configuration%/transform.eig.txt $(DATA_DIR)/configuration% 
	$(VISUALIZE) $(DATA_DIR)/configuration$*/unstructured/camera* $< dynamic/configuration$*/sync.eig.txt

visualize_ground_truth-%: ground_truth/configuration%/transform.eig.txt $(DATA_DIR)/configuration% 
	$(VISUALIZE) $(DATA_DIR)/configuration$*/unstructured/camera* $< ground_truth/configuration$*/sync.eig.txt

visualize_keypoint-%: keypoint/configuration%/transform.eig.txt $(DATA_DIR)/configuration% 
	$(VISUALIZE) $(DATA_DIR)/configuration$*/unstructured/camera* $< keypoint/configuration$*/sync.eig.txt

ground_truth_sync-%: $(DATA_DIR)/configuration%
	rosrun xpl_calibration offset_maker $(DATA_DIR)/configuration$*/unstructured/camera* ground_truth/configuration$*/sync.eig.txt ground_truth/configuration$*/sync.eig.txt

dynamic/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
	cd $(@D) && if [ -e $(MAKE_ROOT)/custom_pipelines/dynamic.pl ]; then \
	echo Using custom pipeline $(MAKE_ROOT)/custom_pipelines/dynamic.pl; \
	cat $(MAKE_ROOT)/custom_pipelines/dynamic.pl; \
	$(CALIBRATE_DYNAMIC) $</unstructured/camera* $(MAKE_ROOT)/custom_pipelines/dynamic.pl; \
	else \
	echo Using programmatically generated pipeline.; \
	$(CALIBRATE_DYNAMIC) $</unstructured/camera*; \
	fi &> transform.eig.txt-log.txt

MAX_DT := 0.015
ground_truth/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
	rosrun cloud_calibration checker_calibrate $(DATA_DIR)/$*/checkerboard/camera* $@ $(MAX_DT) > $@-log.txt
	echo "0" > $(@D)/sync.eig.txt #NO syncing for now

ground_truth_user/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
	rosrun cloud_calibration user_calibrate $(DATA_DIR)/$*/checkerboard/camera* $@ $(MAX_DT) > $@-log.txt
	rosrun xpl_calibration offset_maker $(DATA_DIR)/$*/unstructured/camera* $(@D)/sync.eig.txt > $(@D)/sync.eig.txt-log.txt

keypoint/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
	cd $(@D) &&	$(CALIBRATE_ORB) $(DATA_DIR)/$*/unstructured/camera* > transform.eig.txt-log.txt
	echo "0" > $(@D)/sync.eig.txt #NO syncing for now

evaluations/%.txt: ground_truth/%/transform.eig.txt dynamic/%/transform.eig.txt
	mkdir -p $(@D)
	$(EVALUATE) $^ > $@

############################################################
# -- Plots
############################################################


############################################################
# -- Latex
############################################################

