
############################################################
# -- Global settings.
############################################################

SHELL			:= /bin/bash
RACE_ROOT		:= ../../..
export NUM_THREADS	:= 40
export ROS_PACKAGE_PATH	:=$(ROS_PACKAGE_PATH):$(RACE_ROOT)/src/ros-pkg

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.


SEQUENCE_DIR := /home/teichman/sync/projects/ds\&t/xpl/sequences

TRAINING	:=	dinner_table_trivial \
			magazine \
			trivial \
			mug2_labeled \
			popcorn_trivial

TESTING		:= 	clementine \
			mug1_first15

TRAINING_PATHS	:= 	$(addprefix $(SEQUENCE_DIR)/, $(TRAINING))
TESTING_PATHS	:= 	$(addprefix $(SEQUENCE_DIR)/, $(TESTING))


all:
	@echo You should probably be more specific.


############################################################
# -- Programs
############################################################

TRAIN := NO_MASK= rosrun dst train
EVALUATE := NO_MASK= rosrun dst evaluate


############################################################
# -- Params
############################################################


############################################################
# -- Latex
############################################################

latex/rss2012.pdf: 	latex/rss2012.tex \
			latex/rss2012.bib
	cd $(@D) && pdflatex rss2012
	cd $(@D) && bibtex rss2012
	cd $(@D) && pdflatex rss2012
	cd $(@D) && pdflatex rss2012

############################################################
# -- Experiments
############################################################

%-experiment_metadata.txt:
	mkdir -p $(@D)
	echo `hostname` > $@
	echo `date +%F_%T` >> $@
	echo '' >> $@
	hg tip >> $@
	echo '' >> $@
	env >> $@

display_results_grid_cvpr: all_results_cvpr
	@echo "CVPR training method."
	@echo "FTF Training: " $$(cat ftf_training_results.txt | grep 'Mean Hamming' | awk '{print $$NF}')
	@echo "FTF Testing: " $$(cat ftf_testing_results.txt | grep 'Mean Hamming' | awk '{print $$NF}')
	@echo "Seq Training: " $$(cat training_results.txt | grep 'Mean Hamming' | awk '{print $$NF}')
	@echo "Seq Testing: " $$(cat testing_results.txt | grep 'Mean Hamming' | awk '{print $$NF}')

display_results_grid_robust: all_results_robust
	@echo "Robust training method."
	@echo "FTF Training: " $$(cat ftf_training_results.txt | grep 'Mean Hamming' | awk '{print $$NF}')
	@echo "FTF Testing: " $$(cat ftf_testing_results.txt | grep 'Mean Hamming' | awk '{print $$NF}')
	@echo "Seq Training: " $$(cat training_results.txt | grep 'Mean Hamming' | awk '{print $$NF}')
	@echo "Seq Testing: " $$(cat testing_results.txt | grep 'Mean Hamming' | awk '{print $$NF}')

all_results_robust: 	training_results-robust.txt \
			ftf_training_results-robust.txt \
			testing_results-robust.txt \
			ftf_testing_results-robust.txt

testing_results-robust.txt: weights-robust.eig.txt $$@-experiment_metadata.txt
	$(EVALUATE) weights-robust.eig.txt $(TESTING_PATHS) $@ > $@-log.txt

ftf_testing_results-robust.txt: weights-robust.eig.txt $$@-experiment_metadata.txt
	FTF= $(EVALUATE) weights-robust.eig.txt $(TESTING_PATHS) $@ > $@-log.txt

training_results-robust.txt: weights-robust.eig.txt $$@-experiment_metadata.txt
	$(EVALUATE) weights-robust.eig.txt $(TRAINING_PATHS) $@ > $@-log.txt

ftf_training_results-robust.txt: weights-robust.eig.txt $$@-experiment_metadata.txt
	FTF= $(EVALUATE) weights-robust.eig.txt $(TRAINING_PATHS) $@ > $@-log.txt

weights-robust.eig.txt: $$@-experiment_metadata.txt
	ROBUST= $(TRAIN) $(TRAINING_PATHS) $@ > $@-log.txt


all_results_cvpr: 	training_results.txt \
			ftf_training_results.txt \
			testing_results.txt \
			ftf_testing_results.txt

testing_results.txt: weights.eig.txt $$@-experiment_metadata.txt
	$(EVALUATE) weights.eig.txt $(TESTING_PATHS) $@ > $@-log.txt

ftf_testing_results.txt: weights.eig.txt $$@-experiment_metadata.txt
	FTF= $(EVALUATE) weights.eig.txt $(TESTING_PATHS) $@ > $@-log.txt

training_results.txt: weights.eig.txt $$@-experiment_metadata.txt
	$(EVALUATE) weights.eig.txt $(TRAINING_PATHS) $@ > $@-log.txt

ftf_training_results.txt: weights.eig.txt $$@-experiment_metadata.txt
	FTF= $(EVALUATE) weights.eig.txt $(TRAINING_PATHS) $@ > $@-log.txt

weights.eig.txt: $$@-experiment_metadata.txt
	$(TRAIN) $(TRAINING_PATHS) $@ > $@-log.txt

clean:
	-rm *.eig.txt ftf_training_results.txt training_results.txt ftf_testing_results.txt testing_results.txt
	-rm *-experiment_metadata.txt
	-rm *-log.txt