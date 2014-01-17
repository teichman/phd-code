############################################################
# -- Global settings.
############################################################

SHELL                   := /bin/bash
REPO_ROOT               := $(shell dirname $$(dirname $$(dirname $$(rospack find jarvis))))
MAKE_ROOT               := $(REPO_ROOT)/project/jarvis
MAKEFILE                := $(MAKE_ROOT)/Makefile
export NUM_THREADS      := 24
export ROS_PACKAGE_PATH := $(ROS_PACKAGE_PATH):$(REPO_ROOT)/ros-pkg
ROSRUN			:= rosrun
#ROSRUN			:= . ~/.bashrc && rosgdb

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
CONFIG := `rospack find jarvis`/config/less_gravity.yml
#CONFIG := `rospack find jarvis`/config/hand_specified.yml
CONFIG2 := `rospack find jarvis`/config/old_hand_specified.yml

KITCHEN_DIR := $(LOG_DIR)/kitchen-new_bg_subtr-set0
KITCHEN_NAMES := $(shell ls $(KITCHEN_DIR) | sed 's/\.bag//')
KITCHEN_TD_DIRS := $(addprefix kitchen/tds/, $(KITCHEN_NAMES))
KITCHEN_LABELS_DIR := $(LABELED_TD_DIR)/kitchen
# This is for logfiles taken in the kitchen before ~ Nov 21 2013
KITCHEN_UP_VECTOR := `rospack find jarvis`/config/up-kitchen.eig.txt

KITCHEN_GLUED_DIR := $(LOG_DIR)/kitchen_glued-set0
KITCHEN_GLUED_NAMES := $(shell ls $(KITCHEN_GLUED_DIR) | sed 's/\.bag//')
KITCHEN_GLUED_TD_DIRS := $(addprefix kitchen_glued/tds/, $(KITCHEN_GLUED_NAMES))
KITCHEN_GLUED_LABELS_DIR := $(LABELED_TD_DIR)/kitchen_glued
# This is for logfiles taken in the kitchen after ~ Nov 21 2013
KITCHEN_GLUED_UP_VECTOR := `rospack find jarvis`/config/up-kitchen-post_gluing.eig.txt

FRONT_DOOR_DIR := $(LOG_DIR)/front_door-set0
FRONT_DOOR_NAMES := $(shell ls $(FRONT_DOOR_DIR) | sed 's/\.bag//')
FRONT_DOOR_TD_DIRS := $(addprefix front_door/tds/, $(FRONT_DOOR_NAMES))
FRONT_DOOR_LABELS_DIR := $(LABELED_TD_DIR)/front_door
FRONT_DOOR_UP_VECTOR := `rospack find jarvis`/config/2013-12-12_front_door-up.eig.txt

FRONT_DOOR_DIR_1 := $(LOG_DIR)/front_door-set1
FRONT_DOOR_NAMES_1 := $(shell ls $(FRONT_DOOR_DIR_1) | sed 's/\.bag//')
FRONT_DOOR_TD_DIRS_1 := $(addprefix front_door-set1/tds/, $(FRONT_DOOR_NAMES_1))
FRONT_DOOR_LABELS_DIR_1 := $(LABELED_TD_DIR)/front_door
FRONT_DOOR_UP_VECTOR_1 := `rospack find jarvis`/config/2014-01-14_front_door-up.eig.txt

OFFICE_DIR := $(LOG_DIR)/office-set0
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

kitchen_glued-tds: $(addsuffix /extracted, $(KITCHEN_GLUED_TD_DIRS))
kitchen_glued/tds/%/extracted: $(KITCHEN_GLUED_DIR)/%.bag
	mkdir -p $(@D)
	ROS_MASTER_URI=http://localhost:$$(( $$(ls $(KITCHEN_GLUED_DIR) | awk '{if($$0 == "$*.bag") print NR}') + 11311)) roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
	touch $@

office-tds: $(addsuffix /extracted, $(OFFICE_TD_DIRS))
office/tds/%/extracted: $(OFFICE_DIR)/%.bag
	mkdir -p $(@D)
	roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
	touch $@

############################################################
# front_door-set0
############################################################

FRONT_DOOR_CLASS_NAMES := open_door cat person mailman

front_door-tds: $(addsuffix /extracted, $(FRONT_DOOR_TD_DIRS))
front_door/tds/%/extracted: $(FRONT_DOOR_DIR)/%.bag
	mkdir -p $(@D)
	ROS_MASTER_URI=http://localhost:$$(( $$(ls $(FRONT_DOOR_DIR) | awk '{if($$0 == "$*.bag") print NR}') + 11311)) roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
	touch $@

front_door-update-unlabeled-descriptors: $(addprefix front_door/tds/, $(addsuffix /updated, $(FRONT_DOOR_NAMES)))
front_door/tds/%/updated: front_door/tds/$$*/extracted
	$(ROSRUN) jarvis update_descriptors \
	--config $(CONFIG) \
	-u $(FRONT_DOOR_UP_VECTOR) \
	--tds `find front_door/tds/$*/ -name '*.td' | sort` \
	-j 24
	touch $@

define FRONT_DOOR_RULE_GENERATOR

front_door/$(1)/induction:
	mkdir -p $$@
	$(ROSRUN) jarvis induct \
	--class-names $(1) \
	--config $(CONFIG) \
	-u $(FRONT_DOOR_UP_VECTOR) \
	--emax 0 \
	--buffer-size 1000 \
	--max-track-length 30 \
	--snapshot-every 0 \
	--evaluate-every 1 \
	--test $(FRONT_DOOR_LABELS_DIR)/$(1)/*.td \
	--output-dir $$@ \
	--unlabeled-td-dir front_door/tds/ \
	--init `find front_door/tds/ -name '*.td' | sort -R | head -n5` \
	| tee $$@/log.txt

front_door-save-annotations-$(1):
	$(ROSRUN) online_learning merge_tds \
	--remove-duplicates \
	--tds `find front_door/$(1)/induction/ -name '*.td'` \
	-o `date +%F_%T`-$(1)-annotations.td 

front_door/$(1)/baseline_unfair: front_door/$(1)/induction
	$(ROSRUN) jarvis baseline_unfair \
	--class-names $(1) \
	--config $(CONFIG) \
	-u $(FRONT_DOOR_UP_VECTOR) \
	--root $$< \
	--test $(FRONT_DOOR_LABELS_DIR)/$(1)/*.td \
	--num-iters 15 \
	--o $$@ | tee $$@-log.txt
	mv $$@-log.txt $$@/

front_door/$(1)/group_induction_errors: front_door/$(1)/induction
	$(ROSRUN) online_learning filter_errors \
	-c `find -L $< -name classifier.gc | sort | tail -n1` \
	-d $(FRONT_DOOR_LABELS_DIR)/$(1)/*.td \
	--class-names $(1) \
	-o $$@

compare-front-door-$(1)-results: front_door/$(1)/baseline_unfair
	@grc -es --colour=auto diff -y -W150 \
	`find front_door/$(1)/baseline_unfair/ -name track_results.txt | sort | tail -n1` \
	`find front_door/$(1)/induction/ -name track_results.txt | grep test_results | sort | tail -n1` || exit 0

endef
$(foreach CLASS_NAME,$(FRONT_DOOR_CLASS_NAMES),$(eval $(call FRONT_DOOR_RULE_GENERATOR,$(CLASS_NAME))))

############################################################
# front_door-set1
############################################################

FRONT_DOOR_CLASS_NAMES_1 := open_door cat person mailman

front_door-set1-tds: $(addsuffix /extracted, $(FRONT_DOOR_TD_DIRS_1))
front_door-set1/tds/%/extracted: $(FRONT_DOOR_DIR_1)/%.bag
	mkdir -p $(@D)
	ROS_MASTER_URI=http://localhost:$$(( $$(ls $(FRONT_DOOR_DIR_1) | awk '{if($$0 == "$*.bag") print NR}') + 11311)) roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
	touch $@

front_door-set1-update-unlabeled-descriptors: $(addprefix front_door-set1/tds/, $(addsuffix /updated, $(FRONT_DOOR_NAMES_1)))
front_door-set1/tds/%/updated: front_door-set1/tds/$$*/extracted
	$(ROSRUN) jarvis update_descriptors \
	--config $(CONFIG) \
	-u $(FRONT_DOOR_UP_VECTOR_1) \
	--tds `find front_door-set1/tds/$*/ -name '*.td' | sort` \
	-j 24
	touch $@

define FRONT_DOOR-SET1_RULE_GENERATOR

front_door-set1/$(1)/induction:
	mkdir -p $$@
	$(ROSRUN) jarvis induct \
	--class-names $(1) \
	--config $(CONFIG) \
	-u $(FRONT_DOOR_UP_VECTOR_1) \
	--emax 0 \
	--buffer-size 1000 \
	--max-track-length 30 \
	--snapshot-every 0 \
	--evaluate-every 1 \
	--test $(FRONT_DOOR_LABELS_DIR_1)/$(1)/*.td \
	--output-dir $$@ \
	--unlabeled-td-dir front_door-set1/tds/ \
	--init `find front_door-set1/tds/ -name '*.td' | sort -R | head -n5` \
	| tee $$@/log.txt

front_door-set1-save-annotations-$(1):
	$(ROSRUN) online_learning merge_tds \
	--remove-duplicates \
	--tds `find front_door-set1/$(1)/induction/ -name '*.td'` \
	-o `date +%F_%T`-$(1)-annotations.td 

front_door-set1/$(1)/baseline_unfair: front_door-set1/$(1)/induction
	$(ROSRUN) jarvis baseline_unfair \
	--class-names $(1) \
	--config $(CONFIG) \
	-u $(FRONT_DOOR_UP_VECTOR_1) \
	--root $$< \
	--test $(FRONT_DOOR_LABELS_DIR_1)/$(1)/*.td \
	--num-iters 15 \
	--o $$@ | tee $$@-log.txt
	mv $$@-log.txt $$@/

front_door-set1/$(1)/group_induction_errors: front_door-set1/$(1)/induction
	$(ROSRUN) online_learning filter_errors \
	-c `find -L $$< -name classifier.gc | sort | tail -n1` \
	-d $(FRONT_DOOR_LABELS_DIR_1)/$(1)/*.td \
	--class-names $(1) \
	-o $$@

front_door-set1-update-test-descriptors-$(1):
	$(ROSRUN) jarvis update_descriptors \
	-j 24 \
	-u $(FRONT_DOOR_UP_VECTOR_1) \
	--config $(CONFIG) \
	--tds $(FRONT_DOOR_LABELS_DIR_1)/$(1)/*.td

compare-front-door-set1-$(1)-results: front_door-set1/$(1)/baseline_unfair
	@grc -es --colour=auto diff -y -W150 \
	`find front_door-set1/$(1)/baseline_unfair/ -name track_results.txt | sort | tail -n1` \
	`find front_door-set1/$(1)/induction/ -name track_results.txt | grep test_results | sort | tail -n1` || exit 0

endef
$(foreach CLASS_NAME,$(FRONT_DOOR_CLASS_NAMES_1),$(eval $(call FRONT_DOOR-SET1_RULE_GENERATOR,$(CLASS_NAME))))


############################################################
# kitchen
############################################################

KITCHEN_CLASS_NAMES := cat

kitchen-update-all-descriptors: kitchen-update-unlabeled-descriptors kitchen-update-test-descriptors

kitchen-update-unlabeled-descriptors:
	$(ROSRUN) jarvis update_descriptors \
	--config $(CONFIG) \
	-u $(KITCHEN_UP_VECTOR) \
	--tds `find kitchen/tds/ -name '*.td' | sort` \
	-j 24

kitchen-update-test-descriptors:
	$(ROSRUN) jarvis update_descriptors \
	--config $(CONFIG) \
	-u $(KITCHEN_UP_VECTOR) \
	--tds $(KITCHEN_LABELS_DIR)/*.td \
	-j 24

kitchen/induction: #kitchen-descriptors
	$(ROSRUN) jarvis induct \
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
	kitchen/tds/sentinel_2013-10-24-22-01-00_2/jarvis-0003.td \
	kitchen/tds/sentinel_2013-09-22-18-31-42_1/jarvis-0006.td \
	kitchen/tds/sentinel_2013-09-22-23-15-32_3/jarvis-0005.td \
	kitchen/tds/sentinel_2013-10-22-14-43-33_32/jarvis-0004.td \
	--test $(KITCHEN_LABELS_DIR)/*.td \
	| tee $@-log.txt
	mv $@-log.txt $@/

kitchen/baseline_unfair: kitchen/induction
	$(ROSRUN) jarvis baseline_unfair \
	--class-names $(KITCHEN_CLASS_NAMES) \
	--config $(CONFIG) \
	-u $(KITCHEN_UP_VECTOR) \
	--root $< \
	--test $(KITCHEN_LABELS_DIR)/*.td \
	--num-iters 15 \
	--o $@ | tee $@-log.txt
	mv $@-log.txt $@/

kitchen/cross_evaluation:
	$(ROSRUN) jarvis cross_evaluate \
	--config $(CONFIG) \
	--class-names $(KITCHEN_CLASS_NAMES) \
	--tds $(KITCHEN_LABELS_DIR)/*.td \
	-u $(KITCHEN_UP_VECTOR) \
	-o $@ | tee $@-log.txt
	mv $@-log.txt $@/

kitchen/cross_evaluation2:
	$(ROSRUN) jarvis cross_evaluate \
	--config $(CONFIG2) \
	--class-names $(KITCHEN_CLASS_NAMES) \
	--tds $(KITCHEN_LABELS_DIR)/*.td \
	-u $(KITCHEN_UP_VECTOR) \
	-o $@ | tee $@-log.txt
	mv $@-log.txt $@/

kitchen/group_induction_errors: kitchen/induction
	$(ROSRUN) online_learning filter_errors \
	-c `find kitchen/induction/ -name classifier.gc | sort | tail -n1` \
	-d $(KITCHEN_LABELS_DIR)/*.td \
	--class-names $(KITCHEN_CLASS_NAMES) \
	-o $@


############################################################
# kitchen_glued
############################################################

KITCHEN_GLUED_CLASS_NAMES := cat

kitchen_glued-update-unlabeled-descriptors:
	$(ROSRUN) jarvis update_descriptors \
	--config $(CONFIG) \
	-u $(KITCHEN_GLUED_UP_VECTOR) \
	--tds `find kitchen_glued/tds/ -name '*.td' | sort` \
	-j 24

kitchen_glued/induction: 
	$(ROSRUN) jarvis induct \
	--class-names $(KITCHEN_GLUED_CLASS_NAMES) \
	--config $(CONFIG) \
	-u $(KITCHEN_GLUED_UP_VECTOR) \
	--emax 0 \
	--buffer-size 1000 \
	--max-track-length 30 \
	--snapshot-every 0 \
	--evaluate-every 1 \
	--output-dir $@ \
	--unlabeled-td-dir kitchen_glued/tds/ \
	--init \
	kitchen_glued/tds/sentinel_2013-11-23-19-42-04_8/jarvis-0000.td \
	kitchen_glued/tds/sentinel_2013-11-22-17-07-10_0/jarvis-0000.td \
	kitchen_glued/tds/sentinel_2013-11-24-19-05-46_16/jarvis-0000.td \
	kitchen_glued/tds/sentinel_2013-11-23-19-56-50_9/jarvis-0000.td \
	| tee $@-log.txt
	mv $@-log.txt $@/


############################################################
# Regression tests
############################################################

REGRESSION_TESTS_DIR := $(MAKE_ROOT)/regression_tests
REGRESSION_RUNS_DIR := regression_runs
#REGRESSION_NAMES := $(shell ls $(REGRESSION_TESTS_DIR))
REGRESSION_NAMES := $(shell ls $(REGRESSION_TESTS_DIR) | grep -v kitchen)

all-regression-tests-debug: $(REGRESSION_RUNS_DIR)/completed-debug
$(REGRESSION_RUNS_DIR)/completed-debug: $(addprefix regression-test-,$(REGRESSION_NAMES))
	touch $@

all-regression-tests: $(REGRESSION_RUNS_DIR)/completed
$(REGRESSION_RUNS_DIR)/completed: $(addprefix regression-test-,$(addsuffix -multi, $(REGRESSION_NAMES)))
	touch $@

analyze-regression-tests: $(REGRESSION_RUNS_DIR)/completed
	$(ROSRUN) jarvis analyze_regression_tests.py $(REGRESSION_RUNS_DIR)

define REGRESSION_RULE_GENERATOR
regression-test-$(1)-multi:
	for i in {1..3}; do make -f $(MAKEFILE) regression-test-$(1); done
regression-test-$(1):
	$(ROSRUN) jarvis run_regression.sh $(REGRESSION_TESTS_DIR)/$(1) $(CONFIG) $(REGRESSION_RUNS_DIR)/$(1)
endef
$(foreach REGRESSION_NAME,$(REGRESSION_NAMES),$(eval $(call REGRESSION_RULE_GENERATOR,$(REGRESSION_NAME))))


############################################################
# Twiddling
############################################################

TWIDDLE_TEST_NAMES := kitchen00 leo00 mailman00 open_door00

.PHONY: twiddle
twiddle:
	$(ROSRUN) jarvis twiddle \
	--randomize \
	--decimate 0.5 \
	--initial-config $(CONFIG) \
	--regression-test-dir $(REGRESSION_TESTS_DIR) \
	-o $@ \
	--regression-tests $(TWIDDLE_TEST_NAMES)

.PHONY: twiddle-tiny
twiddle-tiny:
	$(ROSRUN) jarvis twiddle \
	--randomize \
	--decimate 0.9 \
	--initial-config $(CONFIG) \
	--regression-test-dir $(REGRESSION_TESTS_DIR) \
	-o $@ \
	--regression-tests leo00
