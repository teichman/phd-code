
############################################################
# -- Global settings.
############################################################
# You'll need to set REPO_ROOT.
REPO_ROOT		:= /earth/jarvis.git
MAKE_ROOT		:= $(REPO_ROOT)/project/calibration

# You'll need a symlink in your project dir to the data.
# This dir has subdirectories of configuration0, configuration1, etc.
DATA_DIR		:= $(MAKE_ROOT)/data

SHELL			:= /bin/bash
MAKEFILE		:= $(MAKE_ROOT)/Makefile
export NUM_THREADS	:= 8
export ROS_PACKAGE_PATH	:=$(ROS_PACKAGE_PATH):$(REPO_ROOT)/src/ros-pkg

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.


CONFIGURATIONS		:= 	configuration0 \
				configuration1


all:
	@echo You should probably be more specific.


CALIBRATE_DYNAMIC := DEBUG=1 ON=20 OFF=50 rosrun xpl_calibration calibrate_sequences --dynamic
VISUALIZE := rosrun xpl_calibration calibration_viewer

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

visualize_dynamic-%: dynamic/configuration%/transform.eig.txt $(DATA_DIR)/configuration% 
	$(VISUALIZE) $(DATA_DIR)/configuration$*/unstructured/camera* $< dynamic/configuration$*/sync.eig.txt

dynamic/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
	cd $(@D) && if [ -e $(MAKE_ROOT)/custom_pipelines/dynamic.pl ]; then \
	echo Using custom pipeline $(MAKE_ROOT)/custom_pipelines/dynamic.pl; \
	cat $(MAKE_ROOT)/custom_pipelines/dynamic.pl; \
	$(CALIBRATE_DYNAMIC) $</unstructured/camera* $(MAKE_ROOT)/custom_pipelines/dynamic.pl; \
	else \
	echo Using programmatically generated pipeline.; \
	$(CALIBRATE_DYNAMIC) $</unstructured/camera*; \
	fi > transform.eig.txt-log.txt

ground_truth/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
#	cd $(@D) && rosrun xpl_calibration calibrate_sequences $(DATA_DIR)/$*/unstructured/camera*


############################################################
# -- Plots
############################################################


############################################################
# -- Latex
############################################################

