# Copyright 2014-2021 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = remote
OBJECTS = cameraControl.o configurator.o multicastReceive.o remote.o robotFloor.o

MAKEFLAGS = -j8

# Warning: using -O0 will not generate relevant warnings
CFLAGS = -g3 -O3 -m64 -ansi -std=c++17 -Wall -Wextra -Wpedantic -Werror

LDFLAGS = -pthread -L/usr/local/lib
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs  -lopencv_videoio -lpthread

INC = -I../include -I/usr/include/opencv4

all: $(TARGET)
	@echo "WARNING   : the routing for multicast has to be updated/configured"
	@./$(TARGET)

# use the following when no wired or wireless interface available
lo:
	sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
	sudo ifconfig lo multicast

wlan0:
	sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev wlan0
	sudo ifconfig wlan0 multicast

wlp3s0:
	sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev wlp3s0
	sudo ifconfig wlp3s0 multicast


rdel:
	sudo route del -net 224.0.0.0 netmask 240.0.0.0 dev enp0s31f6 || true
	sudo route del -net 224.0.0.0 netmask 240.0.0.0 dev lo || true
	sudo route del -net 224.0.0.0 netmask 240.0.0.0 dev wlp3s0 || true


%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


clean:
	@sync
	@rm -f *.o $(TARGET)
	@rm -f FloorFileSize1.yaml
	@rm -f *.bin *.log

.PHONY: all lo clean

.SECONDARY:
