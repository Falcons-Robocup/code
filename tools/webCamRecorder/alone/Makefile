# Copyright 2016 Andre Pool, April 2016
# Licensed under the Apache License version 2.0
# You may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

TARGET = webCamRecorder
OBJECTS = webCamRecorder.o cameraControl.o

MAKEFLAGS = -j8

CFLAGS0 = -g3 -O2 -m64 -ansi -Wextra -Wpedantic -Wno-unused-but-set-variable -Wno-unused-parameter
CFLAGS = $(CFLAGS0) -Wall

LDFLAGS = -L/usr/local/lib 
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

INC = -I../include -I../../../packages/vision/include 

all: $(TARGET)
	@./$(TARGET)

%.o: ../src/%.cpp Makefile 
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../../../packages/vision/src/%.cpp Makefile 
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	@sync
	@rm -f *.o $(TARGET)

.PHONY: all clean

.SECONDARY:
