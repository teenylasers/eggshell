# TODO:
# Some of this are duplicate in ../common.mk. Clean up and consolidate after
# this is running as a standalone Makefile.

# TOOLS_DIR is where 3rd party libraries are collected.
ifndef TOOLS_DIR
  TOOLS_DIR := $(HOME)/tools
endif
EIGEN_DIR := $(TOOLS_DIR)/eigen-3.3.4

# Build directories
MY_MAKEFLAGS = -C build -f ../Makefile
BUILD_DIR = build/toolkit

# Compiler config
CC := gcc
CXX := g++

CFLAGS += -I. -I.. -I../../toolkit -Werror -Wall -MMD -Wno-sign-compare
CCFLAGS += $(CFLAGS) -std=gnu++11 -I$(EIGEN_DIR)
CXXFLAGS += -g -Wall -Wextra -pthread -std=c++11
LDFLAGS += -Wl -dead_strip

CC_COMPILE = $(CXX) -c $(CCFLAGS) -o $@ $<
C_COMPILE = $(CC) -c $(CFLAGS) -o $@ $<

OBJ = lcp.o toolkit/error.o toolkit/testing.o

.PHONY: dirs all clean test unittest

dirs :
	mkdir -p $(BUILD_DIR)

all : dirs
	$(MAKE) $(MY_MAKEFLAGS) $(OBJ)

test : all
	$(MAKE) $(MY_MAKEFLAGS) unittest

unittest : $(OBJ)
	$(CXX) $(CCFLAGS) $(LDFLAGS) -o test $(OBJ)
	./test

clean :
	-rm -rf build

%.o : ../%.cc
	$(CC_COMPILE)

%.o : ../%.c
	$(C_COMPILE)

toolkit/%.o : ../../toolkit/%.cc
	$(CC_COMPILE)

toolkit/testing.o : ../../toolkit/testing.cc
	$(CC_COMPILE) -D__TOOLKIT_DEFINE_TESTING_MAIN__
