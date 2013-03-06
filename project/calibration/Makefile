
############################################################
# -- Global settings.
############################################################
export NUM_THREADS	:= 8
SHELL			:= /bin/bash
REPO_ROOT		:= $(shell dirname $$(dirname $$(dirname $$(rospack find xpl_calibration))))
MAKE_ROOT		:= $(REPO_ROOT)/project/calibration
MAKEFILE		:= $(MAKE_ROOT)/Makefile

# You'll need a symlink in your project dir to the data.
# This dir has subdirectories of configuration0, configuration1, etc.
DATA_DIR		:= $(MAKE_ROOT)/data

.SECONDEXPANSION:	# Allows use of automatic vars in prerequisites.
.SECONDARY:		# Prevents deletion of intermediate targets in a chain of implicit rules.

all:
	@echo You should probably be more specific.

# rosmake does not play nicely with -j.
# 'make -j8 code' fails with "make[3]: *** read jobs pipe: No such file or directory.  Stop."
# 'make code' succeeds.
# 'make -l8 code' succeeds.
# TODO: This can probably become rosmake -s.
code:
	rosmake xpl_calibration
	rosmake cloud_calibration

CONFIGURATIONS := $(shell ls $(DATA_DIR) | grep configuration)

CALIBRATE_DYNAMIC := DEBUG=1 rosrun xpl_calibration calibrate_sequences --dynamic
CALIBRATE_ORB := rosrun xpl_calibration calibrate_sequences --orb
VISUALIZE := rosrun xpl_calibration calibration_viewer_vis
EVALUATE := rosrun xpl_calibration evaluate

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

all_dynamic: 	$(addprefix dynamic/, $(addsuffix /transform.eig.txt, $(CONFIGURATIONS)))
all_evaluations: $(addprefix evaluations/, $(addsuffix .txt, $(CONFIGURATIONS)))
all_keypoint: $(addprefix keypoint/, $(addsuffix /transform.eig.txt, $(CONFIGURATIONS)))
all_ground_truth: $(addprefix ground_truth/, $(addsuffix /transform.eig.txt, $(CONFIGURATIONS)))
all_ground_truth_user: $(addprefix ground_truth_user/, $(addsuffix /transform.eig.txt, $(CONFIGURATIONS)))

visualize_dynamic-%: dynamic/configuration%/transform.eig.txt $(DATA_DIR)/configuration% 
	$(VISUALIZE) $(DATA_DIR)/configuration$*/unstructured/* $< dynamic/configuration$*/sync.eig.txt

visualize_ground_truth-%: ground_truth/configuration%/transform.eig.txt $(DATA_DIR)/configuration% 
	$(VISUALIZE) $(DATA_DIR)/configuration$*/unstructured/* $< ground_truth/configuration$*/sync.eig.txt

visualize_ground_truth_user-%: ground_truth_user/configuration%/transform.eig.txt $(DATA_DIR)/configuration% 
	$(VISUALIZE) $(DATA_DIR)/configuration$*/unstructured/* $< ground_truth_user/configuration$*/sync.eig.txt

visualize_keypoint-%: keypoint/configuration%/transform.eig.txt $(DATA_DIR)/configuration% 
	$(VISUALIZE) $(DATA_DIR)/configuration$*/unstructured/* $< keypoint/configuration$*/sync.eig.txt

ground_truth_sync-%: $(DATA_DIR)/configuration%
	rosrun xpl_calibration offset_maker $(DATA_DIR)/configuration$*/unstructured/* ground_truth/configuration$*/sync.eig.txt ground_truth/configuration$*/sync.eig.txt

dynamic/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
	cd $(@D) && ON=10 OFF=90 $(CALIBRATE_DYNAMIC) $</unstructured/* | tee transform.eig.txt-log.txt

dynamic-tiny/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
	cd $(@D) && ON=20 OFF=900 $(CALIBRATE_DYNAMIC) $</unstructured/* | tee transform.eig.txt-log.txt

MAX_DT := 0.015
ground_truth/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
	rosrun cloud_calibration checker_calibrate $(DATA_DIR)/$*/checkerboard/* $@ $(MAX_DT) > $@-log.txt

ground_truth_user/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
	rosrun cloud_calibration user_calibrate $(DATA_DIR)/$*/checkerboard/* $@ $(MAX_DT) > $@-log.txt
	rosrun xpl_calibration offset_maker $(DATA_DIR)/$*/unstructured/* $(@D)/sync.eig.txt > $(@D)/sync.eig.txt-log.txt

ground_truth_user-%: $(DATA_DIR)/configuration%
	mkdir -p $(MAKE_ROOT)/ground_truth_user/configuration$*
	rosrun cloud_calibration user_calibrate $(DATA_DIR)/configuration$*/checkerboard/* ground_truth_user/configuration$*/transform.eig.txt $(MAX_DT)


keypoint/%/transform.eig.txt: $(DATA_DIR)/% $$@-experiment_metadata.txt
	mkdir -p $(@D)
	cd $(@D) &&	$(CALIBRATE_ORB) $(DATA_DIR)/$*/unstructured/* > transform.eig.txt-log.txt

evaluations/%.txt: ground_truth/%/transform.eig.txt dynamic/%/transform.eig.txt
	mkdir -p $(@D)
	$(EVALUATE) $^ ground_truth/$*/sync.eig.txt dynamic/$*/sync.eig.txt
	$(EVALUATE) $^ ground_truth/$*/sync.eig.txt dynamic/$*/sync.eig.txt > $@


############################################################
# -- Plots
############################################################

bargraph.pdf: $(addprefix evaluations/, $(addsuffix .txt, $(CONFIGURATIONS)))
	python $(MAKE_ROOT)/plot_bargraph.py


############################################################
# -- Latex
############################################################

.PHONY: icra2013/img
icra2013/img: icra2013/img/bargraph.pdf

icra2013/img/bargraph.pdf: bargraph.pdf
	mkdir -p $(@D)
	cp $< $@

$(MAKE_ROOT)/icra2013/static_img:
	mkdir -p $(@D)
	rsync -av alex@firetail:~/static_img/* $@/

icra2013/icra2013.pdf: 	$(MAKE_ROOT)/icra2013/icra2013.tex \
			$(MAKE_ROOT)/icra2013/icra2013.bib \
			$(MAKE_ROOT)/icra2013/latex_macros.tex \
			$(MAKE_ROOT)/icra2013/static_img \
			icra2013/img
	mkdir -p $(@D)
	if [ `pwd` != $(MAKE_ROOT) ]; then \
	rsync -av $(MAKE_ROOT)/icra2013/* --exclude='*.tex' --exclude='*.bib' $(@D); \
	cd $(@D); \
	ln -sf $(MAKE_ROOT)/icra2013/icra2013.tex; \
	ln -sf $(MAKE_ROOT)/icra2013/icra2013.bib; \
	ln -sf $(MAKE_ROOT)/icra2013/latex_macros.tex; \
	fi
	cd $(@D) && pdflatex icra2013
	cd $(@D) && bibtex icra2013
	cd $(@D) && pdflatex icra2013
	cd $(@D) && pdflatex icra2013
	cd $(@D) && rm icra2013.{aux,bbl,blg,log,out} latex_macros.aux texput.log

test_make_root:
	if [ `pwd` != $(MAKE_ROOT) ]; then \
	echo In an external workspace.; \
	else \
	echo In MAKE_ROOT.; \
	fi

