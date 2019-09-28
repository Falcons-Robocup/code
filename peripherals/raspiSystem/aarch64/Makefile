# Copyright 2018 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = raspiSystem
OBJECTS = $(TARGET).o raspiRgbToBmp.o

MAKEFLAGS = -j8

ifeq ($(shell uname -m), x86_64)
   CC = aarch64-linux-gnu-gcc
else
   CC = aarch64-unknown-linux-gnu-gcc
endif

# TODO: find best settings for aarch64
# use gnu11 instead of c11 to prevent storage size of 'mreq' isn't known
CFLAGS = -g3 -Ofast -ansi -std=gnu11 -Wall -Wextra -Wpedantic -Wno-unused-but-set-variable

LDFLAGS = 
LDLIBS = -lpthread -lrt

INC = -I../../../packages/raspiControl/include

all: $(TARGET)
	@file $(TARGET)

# rule for building on the raspi
%.o: %.c raspiDefaults.hpp
	$(CC) $(CFLAGS) $(INC) -o $@ -c $<
#	cp $< ~/tmp

%.o: ../src/%.c Makefile ../../../packages/raspiControl/include/raspiDefaults.hpp
	$(CC) $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	@rm -f *.o $(TARGET)
	@sync

.PHONY: all clean

.SECONDARY:
