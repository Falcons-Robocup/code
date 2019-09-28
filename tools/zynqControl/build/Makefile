# Copyright 2017 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = zynqControl
OBJECTS = $(TARGET).o tcp_socket.o zynqGrabConfig.o

MAKEFLAGS = -j8

CFLAGS = -m64 -g3 -Ofast -ansi -std=c++11 -Wall -Wextra -Wpedantic -Wno-unused-but-set-variable

INC = -I../include -I../../../peripherals/zynqGrab/include

LDFLAGS = -pthread -L/usr/local/lib
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

all: $(TARGET)
	@./$<

local: $(TARGET)
	@./$< -e

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../../../peripherals/zynqGrab/src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET) : $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	@sync
	@rm -f *.o $(TARGET)
        

.PHONY: all clean

.SECONDARY:
