# the Makefile wrapper so that I can do out of source builds without having to cd first
# based on Maurice Ling's Makefile in irvine-01-sw and this site
#   http://stackoverflow.com/questions/11143062/getting-cmake-to-build-out-of-source-without-wrapping-scripts
# by Mark Hill

RM    := rm -rf
MKDIR := mkdir -p
CMAKE := cmake
TEST_EXECUTABLE_NAME := droneTest
TEST_MAIN_FILE := flight/FlightTest.cpp

TOOLCHAIN_PREFIX := toolchain_
TOOLCHAIN_INSTALL_PREFIX := install_
NATIVE_TOOLCHAIN_NAME := native
BUILD_DIR_BASE = build_
BUILD_DIR = $(BUILD_DIR_BASE)$(subst $(TOOLCHAIN_PREFIX),,$(TOOLCHAIN_NAME))
SCRIPTS_DIR = scripts
MAKEFILE = $(BUILD_DIR)/Makefile
CONFIG = .config

ifneq ("","$(wildcard $(strip $(CONFIG)))")
include $(CONFIG)
endif
ifdef TOOLCHAIN_NAME
ifneq ("$(TOOLCHAIN_NAME)","$(TOOLCHAIN_PREFIX)$(NATIVE_TOOLCHAIN_NAME)")
TOOLCHAIN_DIR = $(BUILD_DIR)/$(subst _,,$(TOOLCHAIN_PREFIX))/$(TOOLCHAIN_NAME)
CMAKE_TOOLCHAIN_FILE = $(BUILD_DIR)/$(TOOLCHAIN_NAME).cmake
TOOLCHAIN_INSTALL_SCRIPTS_DIR = $(SCRIPTS_DIR)
TOOLCHAIN_INSTALL_SCRIPT = $(TOOLCHAIN_INSTALL_SCRIPTS_DIR)/$(TOOLCHAIN_INSTALL_PREFIX)$(TOOLCHAIN_NAME)
endif
endif

TEST_INSTALL_SCRIPT = $(SCRIPTS_DIR)/installTest
export BUILD_DIR TOOLCHAIN_NAME TOOLCHAIN_DIR CMAKE_TOOLCHAIN_FILE

SUBDIRS = data flight motion drivers orientation sensors
INCLUDE_ROOT = include
EIGEN_DIR = $(INCLUDE_ROOT)/eigen
INCLUDES = $(patsubst %,$(INCLUDE_ROOT)/%,$(SUBDIRS)) $(INCLUDE_ROOT) $(EIGEN_DIR)
PROJECT_LIBS = m rt $(SUBDIRS)

COMMFLAGS = -O2 -g
CFLAGS = $(COMMFLAGS)
CXXFLAGS = $(COMMFLAGS)

EMPTY :=
SPACE := $(EMPTY) $(EMPTY)
define create-cmake-list
$(subst $(SPACE),;,$(strip $1))
endef

SUPPORTED_BOARDS := $(subst $(TOOLCHAIN_INSTALL_PREFIX)$(TOOLCHAIN_PREFIX),,$(foreach config,$(shell find $(TOOLCHAIN_INSTALL_SCRIPTS_DIR) \
	-name '$(TOOLCHAIN_INSTALL_PREFIX)$(TOOLCHAIN_PREFIX)*'),$(shell basename $(config))))
SUPPORTED_BOARDS_CONFIG_TARGETS := $(patsubst %,%-config,$(SUPPORTED_BOARDS))

.PHONY: all install test clean distclean cscope cscope.files tags ctags ycm_config depends $(SUPPORTED_BOARDS_CONFIG_TARGETS) reallyclean


all: $(MAKEFILE)
	$(MAKE) -C $(BUILD_DIR)

depends: $(CONFIG)
	@ touch $(CONFIG)

$(MAKEFILE): $(CONFIG) $(CMAKE_TOOLCHAIN_FILE) | $(BUILD_DIR)
	(cd $(BUILD_DIR) && \
		$(CMAKE) -DSUBDIRS="$(call create-cmake-list,$(SUBDIRS))" \
		-DINCLUDES="$(call create-cmake-list,$(INCLUDES))" \
		-DPROJECT_LIBS="$(call create-cmake-list,$(PROJECT_LIBS))" \
		-DTEST_EXECUTABLE_NAME="$(TEST_EXECUTABLE_NAME)" \
		-DTEST_MAIN_FILE="$(TEST_MAIN_FILE)" \
		-DCFLAGS="$(CFLAGS)" \
		-DCXXFLAGS="$(CXXFLAGS)" \
		-DCMAKE_BUILD_TYPE=Debug \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		-DCMAKE_TOOLCHAIN_FILE=$(subst $(BUILD_DIR)/%,%,$(CMAKE_TOOLCHAIN_FILE_)) ..)

$(CMAKE_TOOLCHAIN_FILE): $(CONFIG) | $(BUILD_DIR)
	@ (sh $(TOOLCHAIN_INSTALL_SCRIPT))

$(BUILD_DIR): $(CONFIG)
	$(MKDIR) $@

$(CONFIG):
	$(error "please build a configuration for your device. available config targets: $(SUPPORTED_BOARDS_CONFIG_TARGETS)")

$(SUPPORTED_BOARDS_CONFIG_TARGETS):
	@ printf "TOOLCHAIN_NAME = $(TOOLCHAIN_PREFIX)$(subst -config,,$@)\n" > $(CONFIG)

install:
	@ (sh $(TEST_INSTALL_SCRIPT))

test: install
	@ (ssh rocket $(TEST_EXECUTABLE_NAME))

clean:
	@ (cd $(BUILD_DIR) && $(MAKE) clean)

distclean:
	@- $(MAKE) --silent -C $(BUILD_DIR) clean || true
	@- $(RM) $(MAKEFILE)
	@- $(RM) $(BUILD_DIR)/src
	@- $(RM) $(BUILD_DIR)/test
	@- $(RM) $(BUILD_DIR)/CMake*
	@- $(RM) $(BUILD_DIR)/cmake.*
	@- $(RM) $(BUILD_DIR)/*.cmake
	@- $(RM) $(BUILD_DIR)/*.txt

reallyclean:
	@- $(RM) build*
	@- $(RM) $(CONFIG)
	@- $(RM) cscope*
	@- $(RM) tags
	@- $(RM) .ycm_extra_conf.py



cscope.files: FIND = find
cscope.files: EXCLUDE_DIRS =
cscope cscope.files: CSCOPE = cscope
ctags tags cscope cscople.files ycm_config: CURDIR = $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
ctags tags: CTAGS = ctags
ycm_config: YCM_GEN_CONFIG = $(HOME)/.vim/bundle/YCM-Generator/config_gen.py

cscope: cscope.files
	$(CSCOPE) -b -q -k

# needs fixing
cscope.files: $(CONFIG)
	$(FIND) $(CURDIR) $(patsubst %,-path "$(CURDIR)/%*" -prune -o,$(EXCLUDE_DIRS)) \
		-path "$(CURDIR)/$(BUILD_DIR_BASE)*"  '!' -path "$(CURDIR)/$(BUILD_DIR)*" -prune -a -name "*.*" -o \
		-name "*.[chxsS]" -o \
		-name "*.cpp" -o \
		-name "*.cc" -o \
		-name "*.hpp" > $(CURDIR)/cscope.files

ctags tags:
	$(CTAGS) --recurse --exclude=$(SCRIPTS_DIR) --exclude=$(BUILD_DIR_BASE)* --exclude="*.js" --languages=C --languages=+C++ --totals $(CURDIR)

COMPILE_COMMANDS_FILE := compile_commands.json
$(COMPILE_COMMANDS_FILE): $(CONFIG) | $(BUILD_DIR)
	ln -srf $(BUILD_DIR)/$(COMPILE_COMMANDS_FILE) $(COMPILE_COMMANDS_FILE)

ycm_config: $(CONFIG) $(COMPILE_COMMANDS_FILE)
	@#;w$(YCM_GEN_CONFIG) -f $(CURDIR)






