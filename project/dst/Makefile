
############################################################
# -- Global settings.
############################################################

SHELL			:= /bin/bash
REPO_ROOT		:= $(shell dirname $$(dirname $$(dirname $$(rospack find dst))))
export NUM_THREADS	:= 40  # No longer comparing with HoughTrack on speed so we can bump up the number of threads.
export SKIP		:= 1
export MASK		:= 1
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

HT_FAIL := 		sitting_still \
			dropping_paper \


TRAINING_PATHS	:= 	$(addprefix $(SEQUENCE_DIR)/, $(TRAINING))
TESTING_PATHS	:= 	$(addprefix $(SEQUENCE_DIR)/, $(TESTING))

specificity:
	@echo You should probably be more specific.


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
# -- Experiments
############################################################

weights.eig.txt: $$@-experiment_metadata.txt
	rosrun dst train $(TRAINING_PATHS) $@ | tee $@-log.txt

#testing_results.txt: weights.eig.txt $$@-experiment_metadata.txt
#	rosrun dst evaluate weights.eig.txt $(TESTING_PATHS) $@ | tee $@-log.txt

define EVALUATION_RULE_GENERATOR
dst/$(1)/eval.txt: weights.eig.txt $$$$@-experiment_metadata.txt
	mkdir -p $$(@D)
	rosrun dst evaluate weights.eig.txt $(SEQUENCE_DIR)/$(1) $$@ | tee $$@-log.txt

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
	cd $$(@D) && rosrun dst evaluate_houghtrack $(SEQUENCE_DIR)/$(1) result > eval.txt
endef
$(foreach VAL,$(TESTING),$(eval $(call EVALUATION_RULE_GENERATOR,$(VAL))))

DST_EVALS := $(addprefix dst/, $(addsuffix /eval.txt, $(TESTING)))
dst/evaluation.txt: $$(DST_EVALS)
	cd $(@D) && cat $(addsuffix /eval.txt, $(TESTING)) > evaluation.txt

HT_EVALS := $(addprefix houghtrack/, $(addsuffix /eval.txt, $(TESTING)))
houghtrack/evaluation.txt: $$(HT_EVALS)
	cd $(@D) && cat $(addsuffix /eval.txt, $(TESTING)) > evaluation.txt

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

dst/timing.txt: dst/evaluation.txt
	@cat `find dst/ -name eval.txt` | grep 'Average time' | awk '{sum += $$NF} END {print sum / NR}' > $@
	@echo "DST average time per frame:" `cat $@` ms

############################################################
# -- Plots
############################################################

bargraph.pdf: dst/evaluation.txt houghtrack/evaluation.txt $$(MAKE_ROOT)/plot_bargraph.py
	python $(MAKE_ROOT)/plot_bargraph.py


############################################################
# -- Latex
############################################################

.PHONY: wafr/img
wafr/img: bargraph.pdf
	mkdir -p $@
	cp $< $@

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
