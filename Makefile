# the Makefile wrapper so that I can do out of source builds without having to cd first
# based on Maurice Ling's Makefile in irvine-01-sw and this site
#   http://stackoverflow.com/questions/11143062/getting-cmake-to-build-out-of-source-without-wrapping-scripts
# by Mark Hill

SHELL := /bin/bash
RM    := rm -rf
MKDIR := mkdir -p
TEST_EXECUTABLE_NAME := rocketTest
CMAKE_TOOLCHAIN_FILE := toolchainRaspberryPi0.cmake

SCRIPTS_DIR := scripts
TOOLCHAIN_INSTALL_SCRIPT := $(SCRIPTS_DIR)/installRPIToolchain
TEST_INSTALL_SCRIPT := $(SCRIPTS_DIR)/installTest

all: ./build/Makefile
	@ $(MAKE) -C build

./build/Makefile: toolchain
	@ (cd build > /dev/null 2>&1 && cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE=$(CMAKE_TOOLCHAIN_FILE) ..)

toolchain:
	@ (sh $(TOOLCHAIN_INSTALL_SCRIPT))

install: all
	@ (sh $(TEST_INSTALL_SCRIPT))

test: install
	@ (ssh rocket $(TEST_EXECUTABLE_NAME))

distclean:
	@  ($(MKDIR) build > /dev/null)
	@  (cd build > /dev/null 2>&1 && cmake .. > /dev/null 2>&1)
	@- $(MAKE) --silent -C build clean || true
	@- $(RM) ./build/Makefile
	@- $(RM) ./build/src
	@- $(RM) ./build/test
	@- $(RM) ./build/CMake*
	@- $(RM) ./build/cmake.*
	@- $(RM) ./build/*.cmake
	@- $(RM) ./build/*.txt

clean:
	@ (cd build && $(MAKE) clean)





