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
CONFIG := `rospack find jarvis`/config/config.yml

KITCHEN_DIR := $(LOG_DIR)/kitchen-new_bg_subtr
KITCHEN_NAMES := $(shell ls $(KITCHEN_DIR) | sed 's/\.bag//')
KITCHEN_TD_DIRS := $(addprefix kitchen/tds/, $(KITCHEN_NAMES))
KITCHEN_LABELS_DIR := $(LABELED_TD_DIR)/kitchen

OFFICE_DIR := $(LOG_DIR)/office
OFFICE_NAMES := $(shell ls $(OFFICE_DIR) | sed 's/\.bag//')
OFFICE_TD_DIRS := $(addprefix office/tds/, $(OFFICE_NAMES))
OFFICE_LABELS_DIR := $(LABELED_TD_DIR)/office

debug:
	@echo REPO_ROOT: $(REPO_ROOT)
	@echo MAKE_ROOT: $(MAKE_ROOT)
	@echo LOG_DIR: $(LOG_DIR)
	@echo KITCHEN_DIR: $(KITCHEN_DIR)
	@echo KITCHEN_NAMES: $(KITCHEN_NAMES)
	@echo KITCHEN_TD_DIRS: $(KITCHEN_TD_DIRS)


# -- Extract TD files from bagfiles.
kitchen-tds: $(addsuffix /extracted, $(KITCHEN_TD_DIRS))
kitchen/tds/%/extracted: $(KITCHEN_DIR)/%.bag
	mkdir -p $(@D)
	roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
	touch $@

office-tds: $(addsuffix /extracted, $(OFFICE_TD_DIRS))
office/tds/%/extracted: $(OFFICE_DIR)/%.bag
	mkdir -p $(@D)
	roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
	touch $@

# -- Compute descriptors and add the name mapping.
kitchen_descriptors: $(addsuffix /descriptors_computed, $(KITCHEN_TD_DIRS))
kitchen/tds/%/descriptors_computed: kitchen/tds/%/extracted
	rosrun jarvis update_descriptors --config $(CONFIG) --tds $(@D)/*.td
	rosrun online_learning apply_cmap -n alex cindy cat --tds $(@D)/*.td
	touch $@

kitchen-update-descriptors:
	rosrun jarvis update_descriptors --config $(CONFIG) --tds `find kitchen/tds/ -name '*.td' | sort`
	rosrun jarvis update_descriptors --config $(CONFIG) --tds `find $(KITCHEN_LABELS_DIR)/ -name '*.td' | sort`

kitchen/induction: kitchen_descriptors
	. ~/.bashrc && rosgdb jarvis induct \
	--config $(CONFIG) \
	--emax 0 \
	--buffer-size 5000 \
	--max-track-length 10 \
	--snapshot-every 0 \
	--evaluate-every 5 \
	--output-dir $@ \
	--unlabeled-td-dir kitchen/tds/ \
	--init $(KITCHEN_LABELS_DIR)/cat_only/bg/*.td \
	--autobg $(KITCHEN_LABELS_DIR)/cat_only/bg/*.td \
	--test $(KITCHEN_LABELS_DIR)/cat_only/*.td \
	| tee $@-log.txt
	mv $@-log.txt $@/