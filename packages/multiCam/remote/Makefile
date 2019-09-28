# Copyright 2014-2018 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = remote
OBJECTS = cameraControl.o configurator.o multicastReceive.o remote.o robotFloor.o

MAKEFLAGS = -j8

# APOX add -Wextra
CFLAGS = -g3 -O0 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Wno-unused-but-set-variable

LDFLAGS = -pthread -L/usr/local/lib
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

ifeq ($(shell uname -m), x86_64)
   CFLAGS += -m64
else
   CFLAGS += -m32
endif

INC = -I../include

all: $(TARGET)
	@./$(TARGET)

# use the following when no wired or wireless interface available
lo:
	sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
	sudo ifconfig lo multicast


%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


clean:
	@sync
	@rm -f *.o $(TARGET)

.PHONY: all lo clean cap list format

.SECONDARY:
