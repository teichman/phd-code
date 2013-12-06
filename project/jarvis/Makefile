############################################################
# -- Global settings.
############################################################

SHELL                   := /bin/bash
REPO_ROOT               := $(shell dirname $$(dirname $$(dirname $$(rospack find jarvis))))
MAKE_ROOT               := $(REPO_ROOT)/project/jarvis
MAKEFILE                := $(MAKE_ROOT)/Makefile
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
CONFIG := `rospack find jarvis`/config/hand_specified.yml
CONFIG2 := `rospack find jarvis`/config/old_hand_specified.yml

KITCHEN_DIR := $(LOG_DIR)/kitchen-new_bg_subtr
KITCHEN_NAMES := $(shell ls $(KITCHEN_DIR) | sed 's/\.bag//')
KITCHEN_TD_DIRS := $(addprefix kitchen/tds/, $(KITCHEN_NAMES))
KITCHEN_LABELS_DIR := $(LABELED_TD_DIR)/kitchen
# This is for logfiles taken in the kitchen before ~ Nov 21 2013
KITCHEN_UP_VECTOR := `rospack find jarvis`/config/up-kitchen.eig.txt

KITCHEN_GLUED_DIR := $(LOG_DIR)/kitchen_glued
KITCHEN_GLUED_NAMES := $(shell ls $(KITCHEN_GLUED_DIR) | sed 's/\.bag//')
KITCHEN_GLUED_TD_DIRS := $(addprefix kitchen_glued/tds/, $(KITCHEN_GLUED_NAMES))
KITCHEN_GLUED_LABELS_DIR := $(LABELED_TD_DIR)/kitchen_glued
# This is for logfiles taken in the kitchen after ~ Nov 21 2013
KITCHEN_GLUED_UP_VECTOR := `rospack find jarvis`/config/up-kitchen-post_gluing.eig.txt

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
# kitchen
############################################################

KITCHEN_CLASS_NAMES := cat

kitchen-update-all-descriptors: kitchen-update-unlabeled-descriptors kitchen-update-test-descriptors

kitchen-update-unlabeled-descriptors:
	rosrun jarvis update_descriptors \
	--config $(CONFIG) \
	-u $(KITCHEN_UP_VECTOR) \
	--tds `find kitchen/tds/ -name '*.td' | sort` \
	-j 24

kitchen-update-test-descriptors:
	rosrun jarvis update_descriptors \
	--config $(CONFIG) \
	-u $(KITCHEN_UP_VECTOR) \
	--tds $(KITCHEN_LABELS_DIR)/*.td \
	-j 24

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
	-o $@ | tee $@-log.txt
	mv $@-log.txt $@/

kitchen/cross_evaluation2:
	rosrun jarvis cross_evaluate \
	--config $(CONFIG2) \
	--class-names $(KITCHEN_CLASS_NAMES) \
	--tds $(KITCHEN_LABELS_DIR)/*.td \
	-u $(KITCHEN_UP_VECTOR) \
	-o $@ | tee $@-log.txt
	mv $@-log.txt $@/

kitchen/group_induction_errors: kitchen/induction
	rosrun online_learning filter_errors \
	-c `find kitchen/induction/ -name classifier.gc | sort | tail -n1` \
	-d $(KITCHEN_LABELS_DIR)/*.td \
	--class-names $(KITCHEN_CLASS_NAMES) \
	-o $@


############################################################
# kitchen_glued
############################################################

KITCHEN_GLUED_CLASS_NAMES := cat

kitchen_glued-update-unlabeled-descriptors:
	rosrun jarvis update_descriptors \
	--config $(CONFIG) \
	-u $(KITCHEN_GLUED_UP_VECTOR) \
	--tds `find kitchen_glued/tds/ -name '*.td' | sort` \
	-j 24

kitchen_glued/induction: 
	. ~/.bashrc && rosgdb jarvis induct \
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

REGRESSION_DIR := regression
SAVED_ANNOTATIONS_DIR := $(MAKE_ROOT)/saved_annotations

regression-test-kitchen00-multi:
	for i in {1..10}; do make -f $(MAKEFILE) regression-test-kitchen00; done

regression-test-kitchen00:
	pwd
	RUN_DIR=$(REGRESSION_DIR)/runs/`date +%F_%T`_`cd $(REPO_ROOT) && git rev-parse --short HEAD`_kitchen; \
	mkdir -p $$RUN_DIR/induction; \
	rosrun jarvis induct \
	--randomize \
	--no-vis \
	--max-iters 50 \
	--saved-annotations-dir $(SAVED_ANNOTATIONS_DIR)/kitchen00 \
	--class-names $(KITCHEN_CLASS_NAMES) \
	--config $(CONFIG) \
	-u $(KITCHEN_UP_VECTOR) \
	--emax 0 \
	--buffer-size 1000 \
	--max-track-length 30 \
	--snapshot-every 0 \
	--evaluate-every 1 \
	--output-dir $$RUN_DIR/induction \
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
	| tee $$RUN_DIR/induction/log.txt; \
	cd $$RUN_DIR/induction; \
	rosrun online_learning plot_perclass_pr.py cat; \
	rosrun online_learning plot_perclass_accuracy.py cat; \
	cd -; \
	echo; \
	echo -- Running baseline_unfair --; \
	echo; \
	mkdir -p $$RUN_DIR/baseline_unfair; \
	rosrun jarvis baseline_unfair \
	--class-names $(KITCHEN_CLASS_NAMES) \
	--config $(CONFIG) \
	-u $(KITCHEN_UP_VECTOR) \
	--root $$RUN_DIR/induction \
	--test $(KITCHEN_LABELS_DIR)/*.td \
	--num-iters 10 \
	--o $$RUN_DIR/baseline_unfair \
	| tee $$RUN_DIR/baseline_unfair/log.txt; \
	echo; \
	echo -- Cleaning up. --; \
	echo; \
	cp `find $$RUN_DIR/induction -name classifier.gc | sort | tail -n1` $$RUN_DIR/induction/final_classifier.gc; \
	cp `find $$RUN_DIR/induction -name track_results.txt | sort | tail -n1` $$RUN_DIR/induction/final_track_results.txt; \
	rm `find $$RUN_DIR/induction -name classifier.gc`; \
	rm `find $$RUN_DIR/induction -name 'annotated*.td'`; \
	rm `find $$RUN_DIR/induction -name '*.eig'`; \
	rm `find $$RUN_DIR/induction -name cmap.txt`; \
	rm -rf `find $$RUN_DIR/induction -name test_results_annotated`; \


