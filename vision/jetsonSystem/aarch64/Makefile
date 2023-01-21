# Copyright 2018-2021 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = jetsonSystem
OBJECTS = $(TARGET).o jetsonRgbToBmp.o

MAKEFLAGS = -j8

ifeq ($(shell uname -m), x86_64)
   CC = aarch64-linux-gnu-gcc-10
else
   CC = aarch64-unknown-linux-gnu-gcc
endif

# TODO: try also llvm instead of gcc
# use gnu11 instead of c11 to prevent storage size of 'mreq' isn't known
CFLAGS = -g3 -Ofast -ansi -std=gnu11 -Wall -Wextra -Wpedantic -Wno-unused-but-set-variable 

# TODO: find best settings for aarch64 / ARMv8.2
CFLAGS += -march=armv8.2-a

LDFLAGS = 
LDLIBS = -lpthread -lrt

INC = -I../../../packages/raspiControl/include

all: $(TARGET)
	@file $(TARGET)
	@scp $(TARGET) jetson:
	@ssh jetson ./$(TARGET)

%.o: ../src/%.c Makefile ../../../packages/raspiControl/include/raspiDefaults.hpp
	$(CC) $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	@rm -f *.o $(TARGET)
	@sync

.PHONY: all clean

.SECONDARY:
