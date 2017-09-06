# Copyright 2016 Andre Pool, February 2016
# Licensed under the Apache License version 2.0
# You may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

TARGET = frontVisionAlone
OBJECTS = frontVisionAlone.o frontVision.o configurator.o cameraControl.o

MAKEFLAGS = -j8

CFLAGS0 = -g3 -O2 -m64 -ansi -Wextra -Wpedantic -Wno-unused-but-set-variable -Wno-unused-parameter
CFLAGS = $(CFLAGS0) -Wall

LDFLAGS = -L/usr/local/lib 
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

INC = -I../include 

all: $(TARGET)
	@./$(TARGET)

%.o: ../src/%.cpp ../include/frontVision.hpp Makefile 
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


# show available camera's
list:
	@v4l2-ctl --list-devices


# show camera capabilities
ctl:
	@v4l2-ctl -L --all || true

# show camera resolutions/fps
format:
	@v4l2-ctl --list-formats-ext

clean:
	@sync
	@rm -f *.o $(TARGET)

.PHONY: all clean ctl list format

.SECONDARY:
