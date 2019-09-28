# Copyright 2014-2019 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = raspiSync
OBJECTS = camGrabReceive.o camSysReceive.o raspiControl.o $(TARGET).o

MAKEFLAGS = -j8

CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror

LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

INC = -I../include -I../../../packages/raspiControl/include -I../../../packages/multiCam/include/

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../../../packages/raspiControl/src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../../../packages/multiCam/src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

all: $(TARGET)
	@./$(TARGET)

$(TARGET): $(OBJECTS)
	g++ -o $@ $^ $(LDLIBS)

ETHERNET=`multiCamNet`
eth:
	@echo "## use $(ETHERNET) for the multicast data ##"
	@ip link show $(ETHERNET) | grep -q MULTICAST || sudo ip link set dev $(ETHERNET) multicast on
	@route | grep "224.16.16.0" | grep -q " lo" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 || true
	@route | grep "224.16.16.0" | grep -q " $(ETHERNET)" || sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev $(ETHERNET)


clean:
	@sync
	@rm -f *.o $(TARGET)

.PHONY: all clean eth

.SECONDARY:
