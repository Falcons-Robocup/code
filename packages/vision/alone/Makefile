# Copyright 2014-2016 Andre Pool
# Licensed under the Apache License version 2.0
# You may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

TARGET = visionAlone
OBJECTS = ballDetection.o ballPossession.o cameraControl.o cFrameDiagnostics.o obstacleDetection.o configurator.o determinePosition.o fieldLut.o linePointDetection.o localization.o multicastSend.o preprocessor.o robotFloor.o simplex.o viewer.o visionAlone.o vision.o

MAKEFLAGS = -j8

# APOX add -Wextra
CFLAGS = -g3 -O0 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Wno-unused-but-set-variable -DNOROS

LDFLAGS = -pthread -L/usr/local/lib 
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

ifeq ($(shell uname -m), x86_64)
   CFLAGS += -m64
else
   CFLAGS += -m32
endif

INC = -I../include

all: $(TARGET)
	@nice -n 15 ./$(TARGET) -i $(TURTLE5K_ROBOTNUMBER) -s 2000
#	@./$(TARGET) -i 1 -s 0

demoField: $(TARGET)
	@./$(TARGET) -d dimensionsTechUnited -i 5 -s 10

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)

# show available camera's
list:
	@v4l2-ctl --list-devices


# show camera capabilities
cap:
	@v4l2-ctl -L --all || true

# show camera resolutions/fps
format:
	@v4l2-ctl --list-formats-ext


clean:
	@sync
	@rm -f *.o $(TARGET)

.PHONY: all clean cap list format

.SECONDARY:
