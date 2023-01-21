# Copyright 2014-2021 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = omniCam
OBJECTS = ballDetection.o ballPossession.o cameraControl.o cFrameDiagnostics.o obstacleDetection.o \
   configurator.o determinePosition.o fieldLut.o linePointDetection.o localization.o \
   multicastSend.o preprocessor.o robotFloor.o simplex.o viewer.o visionAlone.o vision.o

MAKEFLAGS = -j8

# Warning: using -O0 will not generate relevant warnings
CFLAGS = -g3 -O3 -m64 -ansi -std=c++17 -Wall -Wextra -Wpedantic -Werror

LDFLAGS = -pthread -L/usr/local/lib
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs  -lopencv_videoio -lpthread

INC = -I../include -I/usr/include/opencv4

all: $(TARGET)
	@./$(TARGET) -i 1 -s 0

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
	@rm -f FloorFileSize1.yaml
	@rm -f *.jpg

.PHONY: all clean cap list format

.SECONDARY:
