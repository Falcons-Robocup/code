# Copyright 2014-2016 Andre Pool
# Licensed under the Apache License version 2.0
# You may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

TARGET = multiCamRemote
OBJECTS = configurator.o multicastReceive.o multiCamRemote.o robotFloor.o

MAKEFLAGS = -j8

CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror

LDFLAGS = -pthread -L/usr/local/lib
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

ifeq ($(shell uname -m), x86_64)
   CFLAGS += -m64
else
   CFLAGS += -m32
endif

INC = -I../include -I../../../packages/multiCam/include/

all: $(TARGET)
	@./$(TARGET)

# use the following when no wired or wireless interface available
lo:
	sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
	sudo ifconfig lo multicast

wlan wlan0:
	sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev wlan0
	sudo ifconfig wlan0 multicast


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
