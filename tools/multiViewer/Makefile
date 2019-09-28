# Copyright 2014-2016 Andre Pool
# Licensed under the Apache License version 2.0
# You may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

TARGET = multiViewer
OBJECTS = $(TARGET).o cameraReceive.o dewarp.o

MAKEFLAGS = -j8

CFLAGS = -m64 -g3 -Ofast -ansi -std=c++11 -Wall -Wextra -Wpedantic -Wno-unused-but-set-variable

LDFLAGS = -pthread -L/usr/local/lib
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

INC = -I. -I../../packages/multiCam/include

all: $(TARGET)
	@./$(TARGET) ../../../data/internal/vision/multiCam/grab0_2018-03-10_18:04:31.jpg ../../packages/multiCam/dewarpConfig0.csv

# use the following when no wired or wireless interface available
lo:
	sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
	sudo ifconfig lo multicast


%.o: %.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: /home/robocup/falcons/code/packages/multiCam/src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


clean:
	@sync
	@rm -f *.o $(TARGET)

.PHONY: all lo clean

.SECONDARY:
