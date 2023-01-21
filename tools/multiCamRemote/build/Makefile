# Copyright 2014-2016 Andre Pool
# Licensed under the Apache License version 2.0
# You may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

TARGET = multiCamRemote
OBJECTS = configurator.o multicastReceive.o multiCamRemote.o robotFloor.o

MAKEFLAGS = -j8

CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror -DNOROS

LDFLAGS = -pthread -L/usr/local/lib -L/home/robocup/falcons/code/build/packages/facilities/common
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

ifeq ($(shell uname -m), x86_64)
   CFLAGS += -m64
else
   CFLAGS += -m32
endif

INC = -I../include -I../../../packages/multiCam/include -I../../../packages/facilities/common/include/ext -I/usr/include/opencv4

all: wlan wlo1 wlp2 wlp3 $(TARGET)
	@./$(TARGET)

# use the following when no wired or wireless interface available
lo:
	sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev lo
	sudo ifconfig lo multicast

wlan wlan0:
	@sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlan0 2>/dev/null || true
	@sudo ifconfig wlan0 multicast 2>/dev/null || true

wlo wlo1:
	@sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlo1 2>/dev/null || true
	@sudo ifconfig wlo1 multicast 2>/dev/null || true

wlp wlp2 wlp2s0:
	@sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlp2s0 2>/dev/null || true
	@sudo ifconfig wlp2s0 multicast 2>/dev/null || true

wlp3 wlp3s0:
	@sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlp3s0 2>/dev/null || true
	@sudo ifconfig wlp3s0 multicast 2>/dev/null || true


%.o: ../../../packages/multiCam/src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


clean:
	@sync
	@rm -f *.o $(TARGET)

.PHONY: all lo clean wlan wlan0

.SECONDARY:
