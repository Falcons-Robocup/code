# Copyright 2018-2021 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = jetsonAnalyze
OBJECTS = $(TARGET).o

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

%.o: ../src/%.c Makefile ../../../packages/raspiControl/include/raspiDefaults.hpp
	$(CC) $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)

ana: eth0 raspiAnalyze
	@ssh cam0 "killall runAnalyze 2>/dev/null || true"
	@ssh cam0 "killall -9 runAnalyze 2>/dev/null || true"
	@ssh cam0 "killall -15 runAnalyze 2>/dev/null || true"
	@ssh cam0 "killall raspiAnalyze 2>/dev/null || true"
	@ssh cam0 "killall -9 raspiAnalyze 2>/dev/null || true"
	@ssh cam0 "killall -15 raspiAnalyze 2>/dev/null || true"
	@ssh cam0 "rm -rf raspiAnalyze; mkdir raspiAnalyze"
	@scp raspiAnalyze cam0:raspiAnalyze
	@ssh -t cam0 "raspiAnalyze/raspiAnalyze -l 0"

eth0:
	@echo
	@echo "## use eth0 for the multicast data ##"
	@ip link show eth0 | grep -q MULTICAST || sudo ip link set dev eth0 multicast on
	@route | grep "224.16.16.0" | grep -q " lo" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 || true
	@route | grep "224.16.16.0" | grep -q " eth0" || sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev eth0

enp0:
	@echo
	@echo "## use enp0s31f6 for the multicast data ##"
	@ip link show enp0s31f6 | grep -q MULTICAST || sudo ip link set dev enp0s31f6 multicast on
	@route | grep "224.16.16.0" | grep -q " lo" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 || true
	@route | grep "224.16.16.0" | grep -q " enp0s31f6" || sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev enp0s31f6


clean:
	@rm -f *.o $(TARGET)
	@sync

.PHONY: all clean ana eth0 enp0

.SECONDARY:
