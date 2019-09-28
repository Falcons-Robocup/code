# Copyright 2018 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = raspiSystem
OBJECTS = $(TARGET).o raspiRgbToBmp.o

MAKEFLAGS = -j8

CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror

LDLIBS = -lpthread

INC = -I../../../packages/raspiControl/include -I../../../packages/multiCam/include

GRAB = /home/robocup/falcons/data/internal/vision/multiCam/r6/grab3_20190428_134730.rgb

all: $(TARGET) valgrind

%.o: ../src/%.c Makefile ../../../packages/raspiControl/include/raspiDefaults.hpp 
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS) md5sumAllFiles.log
	g++ -o $@ $(OBJECTS) $(LDLIBS)

valgrind: $(TARGET)
	valgrind --tool=memcheck --leak-check=full --show-reachable=yes \
	--num-callers=20 --track-fds=yes --track-origins=yes \
	./$(TARGET) -l 2 -t 4 -g $(GRAB) -v

r0:
	./$(TARGET) -l 0 -g ../../raspiAnalyze/x86Build/cam0Image.rgb

r1:
	./$(TARGET) -l 1 -g ../../raspiAnalyze/x86Build/cam1Image.rgb

r2:
	./$(TARGET) -l 2 -g ../../raspiAnalyze/x86Build/cam2Image.rgb

r3:
	./$(TARGET) -l 3 -g ../../raspiAnalyze/x86Build/cam3Image.rgb

md5sumAllFiles.log:
	ln -sf ../../../tools/raspiSetup/md5sumAllFiles.log md5sumAllFiles.log 
 
clean:
	@rm -f *.o $(TARGET) recv md5sumAllFiles.log
	@sync

.PHONY: all clean run valgrind r0 r1 r2 r3

.SECONDARY:
