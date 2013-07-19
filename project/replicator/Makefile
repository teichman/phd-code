############################################################
# -- Global settings
############################################################

# Allows use of automatic vars in prerequisites.
.SECONDEXPANSION:	
# Prevents deletion of intermediate targets in a chain of implicit rules.
.PRECIOUS:
# ... but if there's an error, the target should be deleted.
# Otherwise, the next time you run make, it will try to proceed using that file!
.DELETE_ON_ERROR:	

SHELL			:= /bin/bash
REPO_ROOT		:= $(shell dirname $$(dirname $$(dirname $$(rospack find dst))))
MAKE_ROOT		:= $(REPO_ROOT)/project/replicator
MAKEFILE		:= $(MAKE_ROOT)/Makefile

export NUM_THREADS	:= 40

############################################################
# -- Project data
############################################################

DATA_PATH		:= $(MAKE_ROOT)/data
BUSTS_DATA_PATH		:= $(DATA_PATH)/busts

BUSTS			:= 	xpl00-alex-0 \
				xpl00-alex-1 \
				xpl00-stephen-0 \
				xpl00-stephen-1


############################################################
# -- Busts
############################################################


all-busts: $(addprefix busts/slam_results/, $(BUSTS))

define BUST_RULE_GENERATOR

busts-visualize-bust-$(1): busts/selected/$(1).pcd
	rosrun pcl_bleeding pcd_viewer $$<

busts/selected/$(1).pcd: busts/slam_results/$(1)
	mkdir -p $$(@D)
	rosrun replicator select_box $(BUSTS_DATA_PATH)/$(1) busts/slam_results/$(1)/submap_0.traj -o $$@

busts-visualize-trajectory-$(1): busts/slam_results/$(1)
	rosrun xpl_calibration calibrate_slam --only-visualize --sseq $(BUSTS_DATA_PATH)/$(1) --traj busts/slam_results/$(1)/submap_0.traj

busts-visualize-unrefined-trajectory-$(1): busts/slam_results/$(1)
	rosrun xpl_calibration calibrate_slam --only-visualize --sseq $(BUSTS_DATA_PATH)/$(1) --traj busts/slam_results/$(1)_unrefined/submap_0.traj

busts-view-links-$(1): busts/slam_results/$(1)
	rosrun xpl_calibration view_links $(BUSTS_DATA_PATH)/$(1) $$</graph.log

busts/slam_results/$(1): busts/slam_results/$(1)_unrefined
	rosrun xpl_calibration inspect_pose_graph \
	$(BUSTS_DATA_PATH)/$(1) $$</graph.log --otraj $$@ --opcd $$@ --ograph $$@graph.log

busts-view-unrefined-links-$(1): busts/slam_results/$(1)_unrefined
	rosrun xpl_calibration view_links $(BUSTS_DATA_PATH)/$(1) $$</graph.log

busts/slam_results/$(1)_unrefined: $(BUSTS_DATA_PATH)/$(1)
	mkdir -p $$(@D)
	rosrun xpl_calibration primesense_slam $$< --otraj $$@ --opcd $$@ --ograph $$@/graph.log

endef
$(foreach BUST,$(BUSTS),$(eval $(call BUST_RULE_GENERATOR,$(BUST))))