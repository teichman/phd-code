
############################################################
# -- Global settings
############################################################

SHELL			:= /bin/bash
REPO_ROOT		:= $(shell dirname $$(dirname $$(dirname $$(rospack find dst))))
export NUM_THREADS	:= 40  # No longer comparing with HoughTrack on speed so we can bump up the number of threads.
MAKE_ROOT		:= $(REPO_ROOT)/project/dst
MAKEFILE		:= $(MAKE_ROOT)/Makefile

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.

# These must exist.
SEQUENCE_DIR := /home/teichman/sync/projects/dst/xpl/sequences

TRAINING	:=	dinner_table_trivial \
			magazine \
			trivial \
			popcorn_trivial \
			mug1_first15 \
			boxing_mitt \
			ND_hat_first30 \
			romeo_first19 \
			laptop \
			flat

TESTING		:= 	clementine \
			boxing_glove \
			ND_megaphone \
			manipulating_paper \
			moving_jacket \
			red_still \
			mug2_first200 \
			red_wiggle \
			tongue_flick \
			fingers \
			wghat \
			faces \
			mug_drink \
			nutella633 \
			hbog1 \
			hbog2 \
			hbog3 \
			hung_jacket

# -- Tiny datasets for testing code.
# TRAINING :=	mug1_first15 \
# 		romeo_first19

# TESTING :=	boxing_glove \
# 		boxing_mitt \
# 		ND_megaphone

HT_FAIL := 		sitting_still \
			dropping_paper \


TRAINING_PATHS	:= 	$(addprefix $(SEQUENCE_DIR)/, $(TRAINING))
TESTING_PATHS	:= 	$(addprefix $(SEQUENCE_DIR)/, $(TESTING))

all:
	@echo You should probably be more specific.


EVALUATE	:= rosrun dst evaluate
TRAIN		:= rosrun dst train


############################################################
# -- T-ASE paper
############################################################

tase2013.pdf: 	bargraph.pdf \
		avs/realtime.pdf \
		avs/timing.pdf \
		random/random.pdf \
		$$(MAKE_ROOT)/tase/tase2013.tex

	ln -sf `pwd`/bargraph.pdf $(MAKE_ROOT)/tase/img/
	ln -sf `pwd`/avs/realtime.pdf $(MAKE_ROOT)/tase/img/
	ln -sf `pwd`/avs/timing.pdf $(MAKE_ROOT)/tase/img/
	ln -sf `pwd`/random/random.pdf $(MAKE_ROOT)/tase/img/
	cd $(MAKE_ROOT)/tase && pdflatex tase2013
	cd $(MAKE_ROOT)/tase && bibtex tase2013
	cd $(MAKE_ROOT)/tase && pdflatex tase2013
	cd $(MAKE_ROOT)/tase && pdflatex tase2013
	cd $(MAKE_ROOT)/tase && rm tase2013.{aux,bbl,blg,log,idx} latex_macros.aux
	mv $(MAKE_ROOT)/tase/tase2013.pdf .


############################################################
# -- Plots
############################################################

.PHONY: wafr-results
wafr-results: bargraph.pdf

.PHONY: tase-results
tase-results: bargraph.pdf avs/realtime.pdf avs/timing.pdf random/random.pdf


############################################################
# -- TASE learning
############################################################

random/done:
	mkdir -p $(@D)
	cd $(@D) && for i in {1..1000}; do \
	dir=`date +%F_%T`; \
	mkdir $$dir; cd $$dir; \
	rosrun dst generate_random_weights 18 random.eig.txt; \
	MASK=1 SKIP=2 $(EVALUATE) random.eig.txt $(TESTING_PATHS) testing_results.txt | tee testing_results.txt-log.txt; \
	cd ../; \
	done;
	touch $@


############################################################
# -- TASE realtime experiments
############################################################

avs/timing.pdf: $$(MAKE_ROOT)/plot_timing.py
	python $< $(@D) $(MAKE_ROOT)/vga/rg

avs/accuracy_vs_speed.pdf: 	avs/done \
				$$(MAKE_ROOT)/plot_accuracy_vs_speed.py \
				$$(MAKE_ROOT)/common_plotting.py
	cd $(@D) && \
	for type in mask0_skip01 $(addprefix mask1_skip, 01 02 04 07 99); do \
	echo $$type; \
	cat `find -name testing_results.txt | sort | grep $$type` | grep 'Total time' | awk '{print $$NF}' > $${type}-times.txt; \
	cat `find -name testing_results.txt | sort | grep $$type` | grep 'Overall mean' | awk '{print $$NF}' > $${type}-losses.txt; \
	paste $${type}-times.txt $${type}-losses.txt > $${type}.txt; \
	rm $${type}-times.txt $${type}-losses.txt; \
	done
	cd $(@D) && python $(MAKE_ROOT)/plot_accuracy_vs_speed.py `find -name 'mask*.txt' | sort`

avs/done: #$$@/$$@-experiment_metadata.txt
	mkdir -p $(@D)
	cd $(@D) && for i in {1..10}; do \
	dir=`date +%F_%T`; \
	mkdir $$dir; cd $$dir; \
	mkdir mask0_skip01; cd mask0_skip01; \
	MASK=0 SKIP=01 make -f $(MAKEFILE) testing_results.txt; \
	cd ../; \
	for skip in 01 02 04 07 10 15 20 99; do \
	mkdir mask1_skip$${skip}; cd mask1_skip$${skip}; \
	MASK=1 SKIP=$${skip} make -f $(MAKEFILE) testing_results.txt; \
	cd ../; \
	done; \
	cd ../; \
	done;
	touch $@

avs/realtime.pdf: $$(@D)/done $$(MAKE_ROOT)/plot_realtime.py
	python $(MAKE_ROOT)/plot_realtime.py $(@D)

testing_results.txt: weights.eig.txt $$@-experiment_metadata.txt
	$(EVALUATE) weights.eig.txt $(TESTING_PATHS) $@ | tee $@-log.txt


############################################################
# -- WAFR HoughTrack comparison experiment
############################################################

dst/timing.txt: dst/evaluation.txt
	@cat `find dst/ -name eval.txt` | grep 'Average time' | awk '{sum += $$NF} END {print sum / NR}' > $@
	@echo "DST average time per frame:" `cat $@` ms

bargraph.pdf: dst/evaluation.txt houghtrack/evaluation.txt $$(MAKE_ROOT)/plot_bargraph.py
	python $(MAKE_ROOT)/plot_bargraph.py

means.txt: houghtrack/evaluation.txt dst/evaluation.txt
	@cat houghtrack/evaluation.txt | grep 'Mean Hamming loss' | awk '{sum += $$NF} END {print "HT mean Hamming loss: " sum / NR}'
	@cat houghtrack/evaluation.txt | grep 'Mean capped normalized accuracy' | awk '{sum += $$NF} END {print "HT mean normalized accuracy: " sum / NR}'
	@cat dst/evaluation.txt | grep 'Mean Hamming loss' | awk '{sum += $$NF} END {print "DST mean Hamming loss: " sum / NR}'
	@cat dst/evaluation.txt | grep 'Mean capped normalized accuracy' | awk '{sum += $$NF} END {print "DST mean normalized accuracy: " sum / NR}'

# Ignores sequences which we had to scale up to prevent HT from crashing.
houghtrack/timing.txt: houghtrack/evaluation.txt
	@for file in `find -name result-log.txt | egrep -v 'clementine|hbog|hung_jacket|red_still'`; do \
	cat $$file | grep AVERAGE -A5 | grep -v AVERAGE | awk '{sum += $$NF} END {print sum}'; \
	done | awk '{sum += $$NF} END {print sum / NR}' > $@
	@echo "HoughTrack average time per frame:" `cat $@` seconds

houghtrack/normalized_accuracy.txt: houghtrack/evaluation.txt
	cat evaluation.txt | grep 'Mean capped normalized acc' | awk '{sum += $$NF} END {print sum / NR}' > $@

DST_EVALS := $(addprefix dst/, $(addsuffix /eval.txt, $(TESTING)))
dst/evaluation.txt: $$(DST_EVALS)
	cd $(@D) && cat $(addsuffix /eval.txt, $(TESTING)) > evaluation.txt

HT_EVALS := $(addprefix houghtrack/, $(addsuffix /eval.txt, $(TESTING)))
houghtrack/evaluation.txt: $$(HT_EVALS)
	cd $(@D) && cat $(addsuffix /eval.txt, $(TESTING)) > evaluation.txt

define EVALUATION_RULE_GENERATOR
dst/$(1)/eval.txt: weights.eig.txt $$$$@-experiment_metadata.txt
	mkdir -p $$(@D)
	$(EVALUATE) weights.eig.txt $(SEQUENCE_DIR)/$(1) $$@ | tee $$@-log.txt

houghtrack/$(1)/conf:
	mkdir -p $$(@D)
	if [ -e $(MAKE_ROOT)/custom_ht_confs/$(1).conf ]; then \
	echo Copying custom conf file $(MAKE_ROOT)/custom_ht_confs/$(1).conf; \
	cp $(MAKE_ROOT)/custom_ht_confs/$(1).conf $$@; \
	else \
	echo Generating conf file.; \
	rosrun dst generate_houghtrack_input $(SEQUENCE_DIR)/$(1) > $$@; \
	fi

houghtrack/$(1)/result: houghtrack/$(1)/conf $$$$@-experiment_metadata.txt
	cd $$(@D) && time rosrun houghtrack Track conf | tee result-log.txt

houghtrack/$(1)/eval.txt: houghtrack/$(1)/result
	cd $$(@D) && $(EVALUATE)_houghtrack $(SEQUENCE_DIR)/$(1) result > eval.txt
endef
$(foreach VAL,$(TESTING),$(eval $(call EVALUATION_RULE_GENERATOR,$(VAL))))

weights.eig.txt: $$@-experiment_metadata.txt
	$(TRAIN) $(TRAINING_PATHS) $@ | tee $@-log.txt


############################################################
# -- Params
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
	git di >> $@

dataset_sizes:
	@echo Training set size:
	@echo Number of training sequences: `echo $(TRAINING_PATHS) | xargs -n1 | wc -l`
	@echo `find $(TRAINING_PATHS) -name '*.pcd' | wc -l` frames.
	@echo Testing set size:
	@echo Number of testing sequences: `echo $(TESTING_PATHS) | xargs -n1 | wc -l`
	@echo `find $(TESTING_PATHS) -name '*.pcd' | wc -l` frames.
	@echo Total dataset size:
	@echo Number of sequences: `echo $(TRAINING_PATHS) $(TESTING_PATHS) | xargs -n1 | wc -l`
	@echo `find $(TRAINING_PATHS) $(TESTING_PATHS) -name '*.pcd' | wc -l` frames.


############################################################
# -- Latex
############################################################

.PHONY: wafr/img
wafr/img: bargraph.pdf
	mkdir -p $@
	cp $< $@

# This is kind of gross.  It leaves a copy of the .tex in the workspace dir, and you might
# get confused and edit it rather than the original, then lose your work.
wafr/wafr2012.pdf: 	$$(MAKE_ROOT)/wafr/wafr2012.tex \
			$$(MAKE_ROOT)/wafr/wafr2012.bib \
			$$(MAKE_ROOT)/wafr/svmult.cls \
			$$(MAKE_ROOT)/wafr/ieee.bst \
			$$(MAKE_ROOT)/wafr/latex_macros.tex \
			wafr/img
	mkdir -p $(@D)
	cd $(@D) && cp $(MAKE_ROOT)/wafr/{wafr2012.tex,wafr2012.bib,svmult.cls,ieee.bst,latex_macros.tex} .
	cd $(@D) && pdflatex wafr2012
	cd $(@D) && bibtex wafr2012
	cd $(@D) && pdflatex wafr2012
	cd $(@D) && pdflatex wafr2012
	cd $(@D) && rm wafr2012.{aux,bbl,blg,log,idx}
