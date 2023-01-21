# Copyright 2018-2021 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = jetsonSystem
OBJECTS = $(TARGET).o jetsonRgbToBmp.o

MAKEFLAGS = -j8

CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror

LDLIBS = -lpthread -lrt

GRAB = /home/robocup/falcons/data/internal/vision/multiCam/r6/grab3_20190428_134730.rgb

# TODO: why dependency on multiCam?
INC = -I../../../packages/raspiControl/include

all: $(TARGET) md5sumAllFiles.log
	@./$(TARGET)

%.o: ../src/%.c Makefile ../../../packages/raspiControl/include/raspiDefaults.hpp
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ -o $@ $^ $(LDLIBS)

md5sumAllFiles.log:
	ln -sf ../../../tools/raspiSetup/md5sumAllFiles.log md5sumAllFiles.log 
 
valgrind: $(TARGET)
	valgrind --tool=memcheck --leak-check=full --show-reachable=yes \
	--num-callers=20 --track-fds=yes --track-origins=yes \
	./$(TARGET) -l 2 -t 4 -g $(GRAB) -v

clean:
	@rm -f *.o $(TARGET) md5sumAllFiles.log

.PHONY: all clean run valgrind

.SECONDARY:
