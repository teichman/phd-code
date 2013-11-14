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
CONFIG := `rospack find jarvis`/config/hand_specified-up.yml
CONFIG2 := `rospack find jarvis`/config/old_hand_specified.yml

KITCHEN_DIR := $(LOG_DIR)/kitchen-new_bg_subtr
KITCHEN_NAMES := $(shell ls $(KITCHEN_DIR) | sed 's/\.bag//')
KITCHEN_TD_DIRS := $(addprefix kitchen/tds/, $(KITCHEN_NAMES))
KITCHEN_LABELS_DIR := $(LABELED_TD_DIR)/kitchen
KITCHEN_UP_VECTOR := `rospack find jarvis`/config/up-kitchen.eig.txt

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
	ROS_MASTER_URI=http://localhost:$$(( $$(ls $(KITCHEN_DIR) | awk '{if($$0 == "$*.bag") print NR}') + 11311)) roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
	touch $@

office-tds: $(addsuffix /extracted, $(OFFICE_TD_DIRS))
office/tds/%/extracted: $(OFFICE_DIR)/%.bag
	mkdir -p $(@D)
	roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
	touch $@

# -- Compute descriptors and add the name mapping.
kitchen-td-descriptors: $(addsuffix /descriptors_computed, $(KITCHEN_TD_DIRS))
kitchen/tds/%/descriptors_computed: kitchen/tds/%/extracted
	rosrun jarvis update_descriptors --config $(CONFIG) --tds $(@D)/*.td
	rosrun online_learning apply_cmap -n cat --tds $(@D)/*.td
	touch $@

kitchen-update-all-descriptors: kitchen-update-unlabeled-descriptors kitchen-update-test-descriptors

kitchen-update-unlabeled-descriptors:
	rosrun jarvis update_descriptors --config $(CONFIG) --tds `find kitchen/tds/ -name '*.td' | sort`

kitchen-update-test-descriptors:
	rosrun jarvis update_descriptors \
	--config $(CONFIG) \
	-u $(KITCHEN_UP_VECTOR) \
	--tds $(KITCHEN_LABELS_DIR)/*.td \
	-j 24

KITCHEN_CLASS_NAMES := cat

kitchen/induction: #kitchen-descriptors
	. ~/.bashrc && rosgdb jarvis induct \
	--class-names $(KITCHEN_CLASS_NAMES) \
	--config $(CONFIG) \
	-u $(KITCHEN_UP_VECTOR) \
	--emax 0 \
	--buffer-size 1000 \
	--max-track-length 30 \
	--snapshot-every 0 \
	--evaluate-every 1 \
	--output-dir $@ \
	--unlabeled-td-dir kitchen/tds/ \
	--init \
	kitchen-old/tds/sentinel_2013-09-20-21-00-29_0/jarvis-0003.td \
	kitchen-old/tds/sentinel_2013-09-18-23-31-20_1/jarvis-0003.td \
	kitchen-old/tds/sentinel_2013-09-20-21-00-29_0/jarvis-0001.td \
	kitchen/tds/sentinel_2013-10-25-01-58-21_5/jarvis-0003.td \
	kitchen/tds/sentinel_2013-10-24-19-32-07_1/jarvis-0000.td \
	kitchen/tds/sentinel_2013-10-23-23-04-26_1/jarvis-0005.td \
	kitchen/tds/sentinel_2013-10-26-03-03-44_0/jarvis-0000.td \
	--test $(KITCHEN_LABELS_DIR)/*.td \
	| tee $@-log.txt
	mv $@-log.txt $@/

kitchen/baseline_unfair: kitchen/induction
	rosrun jarvis baseline_unfair \
	--class-names $(KITCHEN_CLASS_NAMES) \
	--config $(CONFIG) \
	-u $(KITCHEN_UP_VECTOR) \
	--root $< \
	--test $(KITCHEN_LABELS_DIR)/*.td \
	--num-iters 15 \
	--o $@ | tee $@-log.txt
	mv $@-log.txt $@/

kitchen/cross_evaluation:
	rosrun jarvis cross_evaluate \
	--config $(CONFIG) \
	--class-names $(KITCHEN_CLASS_NAMES) \
	--tds $(KITCHEN_LABELS_DIR)/*.td \
	-u $(KITCHEN_UP_VECTOR) \
	--num-orderings 3 \
	-o $@ | tee $@-log.txt
	mv $@-log.txt $@/

kitchen/cross_evaluation2:
	rosrun jarvis cross_evaluate \
	--config $(CONFIG2) \
	--class-names $(KITCHEN_CLASS_NAMES) \
	--tds $(KITCHEN_LABELS_DIR)/*.td \
	-u $(KITCHEN_UP_VECTOR) \
	--num-orderings 3 \
	-o $@ | tee $@-log.txt
	mv $@-log.txt $@/

kitchen/group_induction_errors: kitchen/induction
	rosrun online_learning filter_errors \
	-c `find kitchen/induction/ -name classifier.gc | sort | tail -n1` \
	-d $(KITCHEN_LABELS_DIR)/*.td \
	--class-names $(KITCHEN_CLASS_NAMES) \
	-o $@
