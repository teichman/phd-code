############################################################
# -- Global settings.
############################################################

SHELL                   := /bin/bash
REPO_ROOT               := $(shell dirname $$(dirname $$(dirname $$(rospack find jarvis))))
MAKE_ROOT               := $(REPO_ROOT)/project/jarvis
export NUM_THREADS      := 24
export ROS_PACKAGE_PATH := $(ROS_PACKAGE_PATH):$(REPO_ROOT)/ros-pkg

.SECONDEXPANSION:       # Allows use of automatic vars in prerequisites.
.SECONDARY:             # Prevents deletion of intermediate targets in a chain of implicit rules.

all:
	@echo You should probably be more specific.


############################################################
# -- Data setup.
############################################################

# You should symlink these in your own repo.
LOG_DIR := $(MAKE_ROOT)/logs
LABELED_TD_DIR := $(MAKE_ROOT)/labeled

KITCHEN_DIR := $(LOG_DIR)/kitchen-new_bg_subtr
KITCHEN_NAMES := $(shell ls $(KITCHEN_DIR) | sed 's/\.bag//')
KITCHEN_TD_DIRS := $(addprefix kitchen/tds/, $(KITCHEN_NAMES))
KITCHEN_LABELS_DIR := $(LABELED_TD_DIR)/kitchen

debug:
	@echo REPO_ROOT: $(REPO_ROOT)
	@echo MAKE_ROOT: $(MAKE_ROOT)
	@echo LOG_DIR: $(LOG_DIR)
	@echo KITCHEN_DIR: $(KITCHEN_DIR)
	@echo KITCHEN_NAMES: $(KITCHEN_NAMES)
	@echo KITCHEN_TD_DIRS: $(KITCHEN_TD_DIRS)


# -- Extract TD files from bagfiles.
kitchen_tds: $(addsuffix /extracted, $(KITCHEN_TD_DIRS))
kitchen/tds/%/extracted: $(KITCHEN_DIR)/%.bag
	mkdir -p $(@D)
	roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
	touch $@

# -- Compute descriptors and add the name mapping.
kitchen_descriptors: $(addsuffix /descriptors_computed, $(KITCHEN_TD_DIRS))
kitchen/tds/%/descriptors_computed: kitchen/tds/%/extracted
	rosrun jarvis update_descriptors --config `rospack find jarvis`/config/default_descriptor_pipeline.yml --tds $(@D)/*.td
	rosrun online_learning apply_cmap -n alex cindy cat --tds $(@D)/*.td
	touch $@

kitchen/induction: kitchen_descriptors
	rosrun jarvis inductor \
	--emax 0 \
	--buffer-size 5000 \
	--max-track-length 30 \
	--gamma 0 \
	--snapshot-every 50 \
	--evaluate-every 5 \
	--output-dir $@ \
	--unlabeled-td-dir kitchen/tds/ \
	--init \
	./kitchen/tds/sentinel_2013-09-19-01-15-19_2/jarvis-0002.td \
	./kitchen/tds/sentinel_2013-09-20-01-22-17_1/jarvis-0002.td \
	./kitchen/tds/sentinel_2013-09-19-19-29-07_0/jarvis-0002.td \
	--test \
	$(KITCHEN_LABELS_DIR)/*.td \
	--num-cells 10 100 \
	| tee $@-log.txt
	mv $@-log.txt $@/