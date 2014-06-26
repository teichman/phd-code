############################################################
# -- Global settings.
############################################################

SHELL                   := /bin/bash
REPO_ROOT               := $(shell dirname $$(dirname $$(dirname $$(rospack find jarvis))))
MAKE_ROOT               := $(REPO_ROOT)/project/jarvis
MAKEFILE                := $(MAKE_ROOT)/Makefile
export NUM_THREADS      := 24
export ROS_PACKAGE_PATH := $(ROS_PACKAGE_PATH):$(REPO_ROOT)/ros-pkg

ifdef DEBUG
ROSRUN			:= . ~/.bashrc && rosgdb
else
ROSRUN			:= rosrun
endif

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
CONTEXTS := front_door kitchen

# -- Front door variables.
UP_VECTOR_front_door	:= `rospack find jarvis`/config/2013-12-12_front_door-up.eig.txt
CLASS_NAMES_front_door	:= open_door cat person mailman
LOG_DIR_front_door_0	:= $(LOG_DIR)/front_door-set0
LOG_DIR_front_door_1	:= $(LOG_DIR)/front_door-set1
NAMES_front_door_0	:= $(shell ls $(LOG_DIR_front_door_0) | sed 's/\.bag//')
NAMES_front_door_1	:= $(shell ls $(LOG_DIR_front_door_1) | sed 's/\.bag//')
TD_DIRS_front_door_0	:= $(addprefix front_door-set0/tds/, $(NAMES_front_door_0))
TD_DIRS_front_door_1	:= $(addprefix front_door-set1/tds/, $(NAMES_front_door_1))
LABELS_DIR_front_door_0 := $(LABELED_TD_DIR)/front_door-set0
LABELS_DIR_front_door_1 := $(LABELED_TD_DIR)/front_door-set1

# -- Kitchen variables
# This is for logfiles taken in the kitchen before ~ Nov 21 2013
UP_VECTOR_kitchen	:= `rospack find jarvis`/config/up-kitchen.eig.txt
CLASS_NAMES_kitchen	:= cat
LOG_DIR_kitchen_0	:= $(LOG_DIR)/kitchen-new_bg_subtr-set0
LOG_DIR_kitchen_1	:= $(LOG_DIR)/kitchen-new_bg_subtr-set1
NAMES_kitchen_0		:= $(shell ls $(LOG_DIR_kitchen_0) | sed 's/\.bag//')
NAMES_kitchen_1		:= $(shell ls $(LOG_DIR_kitchen_1) | sed 's/\.bag//')
TD_DIRS_kitchen_0	:= $(addprefix kitchen-set0/tds/, $(NAMES_kitchen_0))
TD_DIRS_kitchen_1	:= $(addprefix kitchen-set1/tds/, $(NAMES_kitchen_1))
LABELS_DIR_kitchen_0	:= $(LABELED_TD_DIR)/kitchen-set0
LABELS_DIR_kitchen_1	:= $(LABELED_TD_DIR)/kitchen-set1

# -- Cruft.
KITCHEN_GLUED_DIR := $(LOG_DIR)/kitchen_glued-set0
KITCHEN_GLUED_NAMES := $(shell ls $(KITCHEN_GLUED_DIR) | sed 's/\.bag//')
KITCHEN_GLUED_TD_DIRS := $(addprefix kitchen_glued/tds/, $(KITCHEN_GLUED_NAMES))
KITCHEN_GLUED_LABELS_DIR := $(LABELED_TD_DIR)/kitchen_glued
# This is for logfiles taken in the kitchen after ~ Nov 21 2013
KITCHEN_GLUED_UP_VECTOR := `rospack find jarvis`/config/up-kitchen-post_gluing.eig.txt

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


############################################################
# SET0
############################################################

define SET0_CONTEXT_RULE_GENERATOR

extract-tds-$(1)-set0: $$(addsuffix /extracted, $$(TD_DIRS_$(1)_0))
$(1)-set0/tds/%/extracted: $$(LOG_DIR_$(1)_0)/$$*.bag
	mkdir -p $$(@D)
	ROS_MASTER_URI=http://localhost:$$$$(( $$$$(ls $$(LOG_DIR_$(1)_0) | awk '{if($$$$0 == "$$*.bag") print NR}') + 11311)) roslaunch jarvis extract_tds.launch timeout:=60 bagfile:=$$< output-directory:=$$$$(pwd)/$$(@D)
	touch $$@

update-unlabeled-descriptors-$(1)-set0: $$(addprefix $(1)-set0/tds/, $$(addsuffix /updated, $$(NAMES_$(1)_0)))
$(1)-set0/tds/%/updated: $(1)-set0/tds/$$*/extracted
	$(ROSRUN) jarvis update_descriptors \
	--config $(CONFIG) \
	-u $$(UP_VECTOR_$(1)) \
	--tds `find $(1)-set0/tds/$$*/ -name '*.td' | sort` \
	-j 24
	touch $$@

endef
$(foreach CONTEXT,$(CONTEXTS), \
$(eval $(call SET0_CONTEXT_RULE_GENERATOR,$(CONTEXT))))


define SET0_CONTEXT_CLASS_RULE_GENERATOR

update-test-descriptors-$(1)-set0-$(2):
	$(ROSRUN) jarvis update_descriptors \
	-j 24 \
	-u $$(UP_VECTOR_$(1)) \
	--config $(CONFIG) \
	--tds $$(LABELS_DIR_$(1)_0)/$(2)/*.td

$(1)-set0/$(2)/induction:
	mkdir -p $$@
	$(ROSRUN) jarvis induct \
	--class-names $(2) \
	--config $(CONFIG) \
	-u $$(UP_VECTOR_$(1)) \
	--emax 0 \
	--buffer-size 1000 \
	--max-track-length 30 \
	--snapshot-every 0 \
	--evaluate-every 1 \
	--output-dir $$@ \
	--unlabeled-td-dir $(1)-set0/tds/ \
	--init `find $(1)-set0/tds/ -name '*.td' | sort -R | head -n4` \
	| tee $$@/log.txt

save-annotations-$(1)-set0-$(2):
	$(ROSRUN) online_learning merge_tds \
	--remove-duplicates \
	--tds `find $(1)-set0/$(2)/induction/ -name '*.td'` \
	-o `date +%F_%T`-$(2)-annotations.td 

endef
$(foreach CONTEXT,$(CONTEXTS), \
$(foreach CLASS_NAME,$(CLASS_NAMES_$(CONTEXT)), \
$(eval $(call SET0_CONTEXT_CLASS_RULE_GENERATOR,$(CONTEXT),$(CLASS_NAME)))))


############################################################
# SET1
############################################################

#kitchen-set1/cat/naive_supervised_baseline \
			front_door-set1/cat/naive_supervised_baseline \
			front_door-set1/person/naive_supervised_baseline \
			front_door-set1/mailman/naive_supervised_baseline \
			front_door-set1/open_door/naive_supervised_baseline
archive-manual-runs:
	DIR=manual_runs/kitchen/`date +%F_%T`_`cd $(MAKE_ROOT) && git rev-parse --short HEAD`; \
	mkdir -p $$DIR; \
	rsync -av kitchen-set1/cat/* $$DIR; \
	cp `find $$DIR/induction -name track_results.txt | sort | grep test_results | tail -n1` $$DIR/induction/final_track_results.txt;
	DIR=manual_runs/leo/`date +%F_%T`_`cd $(MAKE_ROOT) && git rev-parse --short HEAD`; \
	mkdir -p $$DIR; \
	rsync -av front_door-set1/cat/* $$DIR; \
	cp `find $$DIR/induction -name track_results.txt | sort | grep test_results | tail -n1` $$DIR/induction/final_track_results.txt;
	DIR=manual_runs/person/`date +%F_%T`_`cd $(MAKE_ROOT) && git rev-parse --short HEAD`; \
	mkdir -p $$DIR; \
	rsync -av front_door-set1/person/* $$DIR; \
	cp `find $$DIR/induction -name track_results.txt | sort | grep test_results | tail -n1` $$DIR/induction/final_track_results.txt;
	DIR=manual_runs/open_door/`date +%F_%T`_`cd $(MAKE_ROOT) && git rev-parse --short HEAD`; \
	mkdir -p $$DIR; \
	rsync -av front_door-set1/open_door/* $$DIR; \
	cp `find $$DIR/induction -name track_results.txt | sort | grep test_results | tail -n1` $$DIR/induction/final_track_results.txt;
	DIR=manual_runs/mailman/`date +%F_%T`_`cd $(MAKE_ROOT) && git rev-parse --short HEAD`; \
	mkdir -p $$DIR; \
	rsync -av front_door-set1/mailman/* $$DIR; \
	cp `find $$DIR/induction -name track_results.txt | sort | grep test_results | tail -n1` $$DIR/induction/final_track_results.txt;

analyze-manual-runs:
	rosrun jarvis analyze_regression_tests.py manual_runs -t nsb

define SET1_CONTEXT_RULE_GENERATOR

extract-tds-$(1)-set1: $$(addsuffix /extracted, $$(TD_DIRS_$(1)_1))
$(1)-set1/tds/%/extracted: $$(LOG_DIR_$(1)_1)/$$*.bag
	mkdir -p $$(@D)
	ROS_MASTER_URI=http://localhost:$$$$(( $$$$(ls $$(LOG_DIR_$(1)_1) | awk '{if($$$$0 == "$$*.bag") print NR}') + 11311)) roslaunch jarvis extract_tds.launch timeout:=60 bagfile:=$$< output-directory:=$$$$(pwd)/$$(@D)
	touch $$@

update-unlabeled-descriptors-$(1)-set1: $$(addprefix $(1)-set1/tds/, $$(addsuffix /updated, $$(NAMES_$(1)_1)))
$(1)-set1/tds/%/updated: $(1)-set1/tds/%/extracted
	$(ROSRUN) jarvis update_descriptors \
	--config $(CONFIG) \
	-u $$(UP_VECTOR_$(1)) \
	--tds `find $(1)-set1/tds/$$*/ -name '*.td' | sort` \
	-j 24
	touch $$@


$(1)-set1-all-test-set-statistics: $$(addprefix $(1)-set1-, $$(addsuffix -test-set-statistics, $$(CLASS_NAMES_$(1))))

endef
$(foreach CONTEXT,$(CONTEXTS), \
$(eval $(call SET1_CONTEXT_RULE_GENERATOR,$(CONTEXT))))


define SET1_CONTEXT_CLASS_RULE_GENERATOR

$(1)-set1-$(2)-test-set-statistics:
	@echo ==================== $(1) $(2) test set summary ====================
	@. ~/.bashrc && $(ROSRUN) online_learning stat $$(LABELS_DIR_$(1)_0)/$(2)/*.td | grep 'tracks\.' | grep -v empty | stats 1

$(1)-set1/$(2)/induction:
	mkdir -p $$@
	$(ROSRUN) jarvis induct \
	--randomize \
	--class-names $(2) \
	--config $(CONFIG) \
	-u $$(UP_VECTOR_$(1)) \
	--emax 0 \
	--buffer-size 1000 \
	--max-track-length 90 \
	--snapshot-every 0 \
	--evaluate-every 1 \
	--test $$(LABELS_DIR_$(1)_0)/$(2)/*.td \
	--output-dir $$@ \
	--unlabeled-td-dir $(1)-set1/tds-lw \
	--init $(1)-set1/tds-lw/*.td \
	| tee $$@/log.txt

baselines-$(1)-set1-$(2): $(1)-set1/$(2)/baseline_unfair $(1)-set1/$(2)/naive_supervised_baseline

save-annotations-$(1)-set1-$(2):
	$(ROSRUN) online_learning merge_tds \
	--remove-duplicates \
	--tds `find $(1)-set1/$(2)/induction/ -name '*.td'` \
	-o `date +%F_%T`-$(2)-annotations.td

clean-induction-dir-$(1)-set1-$(2):
	rm -f `find $(1)-set1/$(2)/induction -name classifier.gc | sort | sed '$$$$d'`
	rm -f `find $(1)-set1/$(2)/induction -name '*.td' | sort`
	rm -f `find $(1)-set1/$(2)/induction -name '*.eig'`
	rm -f `find $(1)-set1/$(2)/induction -name cmap.txt`
	rm -rf `find $(1)-set1/$(2)/induction -name annotation_results | sort`
	rm -rf `find $(1)-set1/$(2)/induction -name validation_results | sort`

$(1)-set1/$(2)/baseline_unfair: $(1)-set1/$(2)/induction
	$(ROSRUN) jarvis baseline_unfair \
	--class-names $(2) \
	--config $(CONFIG) \
	-u $$(UP_VECTOR_$(1)) \
	--root $$< \
	--test $$(LABELS_DIR_$(1)_0)/$(2)/*.td \
	--num-iters 15 \
	--o $$@ | tee $$@-log.txt
	mv $$@-log.txt $$@/

$(1)-set1/$(2)/naive_supervised_baseline: $(1)-set1/$(2)/induction
	mkdir -p $$@
	$(ROSRUN) jarvis naive_supervised_baseline \
	--config $(CONFIG) \
	-u $$(UP_VECTOR_$(1)) \
	--class-names $(2) \
	--train $$(LABELS_DIR_$(1)_1)/$(2)/*.td \
	--test $$(LABELS_DIR_$(1)_0)/$(2)/*.td \
	--num-runs 15 \
	--subsample `grep -A2 'Hand-annotated' $$$$(find $(1)-set1/$(2)/induction -name learner_status.txt | sort | tail -n1) | grep tracks | awk '{print $$$$1}'` \
	-o $$@ \
	-j 24 \
	--randomize | tee $$@/log.txt

$(1)-set1/$(2)/group_induction_errors: $(1)-set1/$(2)/induction
	$(ROSRUN) online_learning filter_errors \
	-c `find -L $$< -name classifier.gc | sort | tail -n1` \
	-d $$(LABELS_DIR_$(1)_0)/$(2)/*.td \
	--class-names $(2) \
	-o $$@

update-test-descriptors-$(1)-set1-$(2):
	$(ROSRUN) jarvis update_descriptors \
	-j 24 \
	-u $$(UP_VECTOR_$(1)) \
	--config $(CONFIG) \
	--tds $$(LABELS_DIR_$(1)_1)/$(2)/*.td

compare-results-$(1)-set1-$(2): $(1)-set1/$(2)/baseline_unfair $(1)-set1/$(2)/naive_supervised_baseline
	@echo -en 'Group induction:\t\t'
	@echo $$$$(grep 'Total acc' `find $(1)-set1/$(2)/induction/ -name track_results.txt | grep test_results | sort | tail -n1` | awk '{print $$$$NF}')
	@echo -en 'Matched supervised baseline:\t'
	@echo $$$$(grep 'Total acc' `find $(1)-set1/$(2)/baseline_unfair/ -name track_results.txt | sort | tail -n1` | awk '{print $$$$NF}')
	@echo -en 'Naive supervised baseline:\t'
	@echo $$$$(grep 'Total acc' $(1)-set1/$(2)/naive_supervised_baseline/average_results/track_results.txt | awk '{print $$$$NF}')
	@echo
	@echo -- Annotations from group induction --
	@grep 'Hand-annotated' -A10 `find $(1)-set1/$(2)/induction/ -name learner_status.txt | sort | tail -n1`

# The || exit 0 is so that Make does not interpret the non-zero return value of diff
# as an error.
compare-results-with-matched-$(1)-set1-$(2): $(1)-set1/$(2)/baseline_unfair
	@grc -es --colour=auto diff -y -W150 \
	`find $(1)-set1/$(2)/baseline_unfair/ -name track_results.txt | sort | tail -n1` \
	`find $(1)-set1/$(2)/induction/ -name track_results.txt | grep test_results | sort | tail -n1` || exit 0

compare-results-with-naive-$(1)-set1-$(2): $(1)-set1/$(2)/naive_supervised_baseline
	@grc -es --colour=auto diff -y -W150 \
	$$</average_results/track_results.txt \
	`find $(1)-set1/$(2)/induction/ -name track_results.txt | grep test_results | sort | tail -n1` || exit 0

endef
$(foreach CONTEXT,$(CONTEXTS), \
$(foreach CLASS_NAME,$(CLASS_NAMES_$(CONTEXT)), \
$(eval $(call SET1_CONTEXT_CLASS_RULE_GENERATOR,$(CONTEXT),$(CLASS_NAME)))))


# ############################################################
# # kitchen
# ############################################################

# KITCHEN_CLASS_NAMES := cat

# kitchen-update-all-descriptors: kitchen-update-unlabeled-descriptors kitchen-update-test-descriptors

# kitchen-update-unlabeled-descriptors:
# 	$(ROSRUN) jarvis update_descriptors \
# 	--config $(CONFIG) \
# 	-u $(KITCHEN_UP_VECTOR) \
# 	--tds `find kitchen/tds/ -name '*.td' | sort` \
# 	-j 24

# kitchen-update-test-descriptors:
# 	$(ROSRUN) jarvis update_descriptors \
# 	--config $(CONFIG) \
# 	-u $(KITCHEN_UP_VECTOR) \
# 	--tds $(KITCHEN_LABELS_DIR)/*.td \
# 	-j 24

# kitchen/induction: #kitchen-descriptors
# 	$(ROSRUN) jarvis induct \
# 	--class-names $(KITCHEN_CLASS_NAMES) \
# 	--config $(CONFIG) \
# 	-u $(KITCHEN_UP_VECTOR) \
# 	--emax 0 \
# 	--buffer-size 1000 \
# 	--max-track-length 30 \
# 	--snapshot-every 0 \
# 	--evaluate-every 1 \
# 	--output-dir $@ \
# 	--unlabeled-td-dir kitchen/tds/ \
# 	--init `find kitchen/tds/ -name '*.td' | sort -R | tail -n4` \
# 	--test $(KITCHEN_LABELS_DIR)/*.td \
# 	| tee $@-log.txt
# 	mv $@-log.txt $@/

# kitchen/baseline_unfair: kitchen/induction
# 	$(ROSRUN) jarvis baseline_unfair \
# 	--class-names $(KITCHEN_CLASS_NAMES) \
# 	--config $(CONFIG) \
# 	-u $(KITCHEN_UP_VECTOR) \
# 	--root $< \
# 	--test $(KITCHEN_LABELS_DIR)/*.td \
# 	--num-iters 15 \
# 	--o $@ | tee $@-log.txt
# 	mv $@-log.txt $@/

# kitchen/cross_evaluation:
# 	$(ROSRUN) jarvis cross_evaluate \
# 	--config $(CONFIG) \
# 	--class-names $(KITCHEN_CLASS_NAMES) \
# 	--tds $(KITCHEN_LABELS_DIR)/*.td \
# 	-u $(KITCHEN_UP_VECTOR) \
# 	-o $@ | tee $@-log.txt
# 	mv $@-log.txt $@/

# kitchen/cross_evaluation2:
# 	$(ROSRUN) jarvis cross_evaluate \
# 	--config $(CONFIG2) \
# 	--class-names $(KITCHEN_CLASS_NAMES) \
# 	--tds $(KITCHEN_LABELS_DIR)/*.td \
# 	-u $(KITCHEN_UP_VECTOR) \
# 	-o $@ | tee $@-log.txt
# 	mv $@-log.txt $@/

# kitchen/group_induction_errors: kitchen/induction
# 	$(ROSRUN) online_learning filter_errors \
# 	-c `find kitchen/induction/ -name classifier.gc | sort | tail -n1` \
# 	-d $(KITCHEN_LABELS_DIR)/*.td \
# 	--class-names $(KITCHEN_CLASS_NAMES) \
# 	-o $@


############################################################
# kitchen_glued
############################################################

kitchen_glued-tds: $(addsuffix /extracted, $(KITCHEN_GLUED_TD_DIRS))
kitchen_glued/tds/%/extracted: $(KITCHEN_GLUED_DIR)/%.bag
	mkdir -p $(@D)
	ROS_MASTER_URI=http://localhost:$$(( $$(ls $(KITCHEN_GLUED_DIR) | awk '{if($$0 == "$*.bag") print NR}') + 11311)) roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
	touch $@

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
REGRESSION_NAMES := kitchen00 leo00 mailman00 open_door00 person00

update-all-regression-test-descriptors-no-up-vector:
	$(ROSRUN) jarvis update_descriptors \
	--force \
	--config $(CONFIG) \
	--tds `find -L $(REGRESSION_TESTS_DIR) -name '*.td' | sort` \
	-j 24

update-all-test-descriptors:   kitchen-set0-update-test-descriptors-cat \
                               front_door-set0-update-test-descriptors-cat \
                               front_door-set0-update-test-descriptors-mailman \
                               front_door-set0-update-test-descriptors-open_door \
                               front_door-set0-update-test-descriptors-person

all-regression-tests-debug: $(REGRESSION_RUNS_DIR)/completed-debug
$(REGRESSION_RUNS_DIR)/completed-debug: $(addprefix regression-test-,$(REGRESSION_NAMES))
	touch $@

all-regression-tests: $(REGRESSION_RUNS_DIR)/completed
$(REGRESSION_RUNS_DIR)/completed: $(addprefix regression-test-,$(addsuffix -multi, $(REGRESSION_NAMES)))
	touch $@

analyze-regression-tests: $(REGRESSION_RUNS_DIR)/completed
	$(ROSRUN) jarvis analyze_regression_tests.py $(REGRESSION_RUNS_DIR)


MAX_ITERS := 40

ifndef NUM_REGRESSION_ITERS
NUM_REGRESSION_ITERS	:= 3
endif

define REGRESSION_RULE_GENERATOR
regression-test-$(1)-multi:
	for i in {1..$(NUM_REGRESSION_ITERS)}; do make -f $(MAKEFILE) regression-test-$(1); done
regression-test-$(1):
	$(ROSRUN) jarvis run_regression.sh $(REGRESSION_TESTS_DIR)/$(1) $(CONFIG) $(MAX_ITERS) $(REGRESSION_RUNS_DIR)/$(1)
endef
$(foreach REGRESSION_NAME,$(REGRESSION_NAMES),$(eval $(call REGRESSION_RULE_GENERATOR,$(REGRESSION_NAME))))


############################################################
# Twiddling
############################################################

TWIDDLE_TEST_NAMES := kitchen00 leo00 mailman00 open_door00 person00

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
