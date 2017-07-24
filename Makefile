# drone sw makefile
# by Mark Hill

TEST_EXECUTABLE_NAME := droneTest
EXECUTABLE_NAME := drone
BUILD_DIR := build

SCRIPTS_DIR := scripts
TOOLCHAIN_INSTALL_SCRIPT := $(SCRIPTS_DIR)/installRPIToolchain
INSTALL_SCRIPT := $(SCRIPTS_DIR)/installTest

CROSS_COMPILE = ""
CC = $(CROSS_COMPILE)cc
CPP = $(CC) -E
CXX = $(CROSS_COMPILE)g++
LD = $(CROSS_COMPILE)ld
AS = $(CROSS_COMPILE)as
AR = $(CROSS_COMPILE)ar
NM = $(CROSS_COMPILE)nm
STRIP = $(CROSS_COMPILE)strip
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMB = $(CROSS_COMPILE)objdump

MKDIR = mkdir
MV = mv
RM = rm

CPPFLAGS = ""
CFLAGS = "-g -O2"
CXXFLAGS = "-g -O2"
LDFLAGS = ""
LDLIBS = ""

export CROSS_COMPILE CC CPP CXX LD AS AR NM STRIP OBJCOPY OBJDUMB CFLAGS CPPFLAGS CXXFLAGS LDFLAGS MKDIR BUILD_DIR MV RM

SUBDIRS := drivers sensors orientation motion flight
SUBOBJS := $(patsubst %, %/%-built-in.o, $(SUBDIRS))
SUBCLEAN := $(patsubst %, %-clean, $(SUBDIRS))

.PHONY : all install test check $(SUBDIRS) $(SUBCLEAN) subdirs subclean clean cscope cscope.files TAGS ctags tags ycm_config

all: subdirs
	$(CC) -o $(EXECUTABLE_NAME) $(LDFLAGS) $(LDLIBS) $(SUBOBJS)

subdirs: $(SUBDIRS) | $(BUILD_DIR)
$(SUBDIRS):
	$(MAKE) $(MAKEFLAGS) -C $@

$(BUILD_DIR):
	$(MKDIR) -p $@

install:
	sh $(INSTALL_SCRIPT)

test check:
	$(TEST_EXECUTABLE_NAME)

clean: subclean
	$(RM) $(EXECUTABLE_NAME) $(TEST_EXECUTABLE_NAME)

subclean: $(SUBCLEAN)
$(SUBCLEAN):
	$(MAKE) -C $(subst -clean,, $@) clean




YCM_GEN_CONFIG = $(HOME)/.vim/bundle/YCM-Generator/config_gen.py 
CTAGS = ctags
CSCOPE = cscope
FIND = find
CURDIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

cscope: cscope.files
	$(CSCOPE) -b -q

# needs fixing
cscope.files:
	$(FIND) $(CURDIR) $(patsubst %, -path $(CURDIR)/%*, $(SUBDIRS)) -name "*.[chxsS]" -name "*.cpp" -name "*.cc" -name "*.hpp" > $(CURDIR)/cscope.files
	
TAGS ctags tags:
	$(CTAGS) --recurse --exclude=$(SCRIPTS_DIR) --exclude=$(BUILD_DIR) --exclude="*.js" --languages=C --languages=+C++ --totals $(CURDIR)

ycm_config:
	$(YCM_GEN_CONFIG) -f $(CURDIR)





