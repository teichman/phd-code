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

# You should symlink this in your own repo.
LOG_DIR := $(MAKE_ROOT)/logs
KITCHEN_DIR := $(LOG_DIR)/kitchen-new_bg_subtr
KITCHEN_NAMES := $(shell ls $(KITCHEN_DIR) | sed 's/\.bag//')
KITCHEN_TD_DIRS := $(addprefix kitchen/tds/, $(KITCHEN_NAMES))

debug:
	@echo REPO_ROOT: $(REPO_ROOT)
	@echo MAKE_ROOT: $(MAKE_ROOT)
	@echo LOG_DIR: $(LOG_DIR)
	@echo KITCHEN_DIR: $(KITCHEN_DIR)
	@echo KITCHEN_NAMES: $(KITCHEN_NAMES)
	@echo KITCHEN_TD_DIRS: $(KITCHEN_TD_DIRS)


# -- Extract TD files from bagfiles.
kitchen_tds: $(addsuffix /done, $(KITCHEN_TD_DIRS))
kitchen/tds/%/done: $(KITCHEN_DIR)/%.bag
	mkdir -p $(@D)
	roslaunch jarvis extract_tds.launch bagfile:=$< output-directory:=$$(pwd)/$(@D)
