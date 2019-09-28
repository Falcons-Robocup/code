# Copyright 2018 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = raspiControlMain
OBJECTS = raspiControl.o camSysReceive.o $(TARGET).o

MAKEFLAGS = -j8

CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror

INC = -I../include -I../../../packages/multiCam/include/

LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

all: $(TARGET)
	@killall $< 2>/dev/null|| killall -9 $< 2>/dev/null|| killall -15 $< 2>/dev/null|| true
	@./$<

%.o: ../src/%.cpp Makefile ../include/raspiDefaults.hpp
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../../../packages/multiCam/src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ -o $@ $^ $(LDLIBS)
	
ETHERNET=`multiCamNet`
eth:
	@echo "## use $(ETHERNET) for the multicast data ##"
	@ip link show $(ETHERNET) | grep -q MULTICAST || sudo ip link set dev $(ETHERNET) multicast on
	@route | grep "224.16.16.0" | grep -q " lo" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 || true
	@route | grep "224.16.16.0" | grep -q " $(ETHERNET)" || sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev $(ETHERNET)

rdel:
	@echo "## use $(ETHERNET) for the multicast data ##"
	@route | grep "224.16.16.0" | grep -q " lo" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 || true
	@route | grep "224.16.16.0" | grep -q " $(ETHERNET)" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 dev $(ETHERNET) || true

clean:
	@sync
	@rm -f *.o $(TARGET)
        

.PHONY: all clean eth rdel

.SECONDARY:
