# Copyright 2018 Andre Pool
# SPDX-License-Identifier: Apache-2.0

# path=~/data
path=~/falcons/data/internal/vision/multiCam

filedate = $(shell date +%Y-%m-%d_%H:%M:%S)
# fileread = 2018-03-10_19:25:30
# fileread = 2018-03-10_19:27:51
# fileread = 2018-03-10_19:27:40
# fileread = 2018-03-10_19:33:54
# fileread = 2018-03-10_19:37:01
# fileread = 2018-03-22_20:00:56
# fileread = 2018-03-22_20:11:17
# fileread = 2018-03-22_20:40:05
# fileread = 2018-03-22_20:41:29
# fileread = 2018-03-22_20:42:26
# fileread = 2018-03-22_20:43:21
# fileread = 2018-04-27_22:42:02
# fileread = 2018-04-29_13:41:14
fileread = 2018-04-29_13:33:21
# fileread = 2018-04-29_13:35:52
# fileread = 2018-04-29_13:37:43

TARGET = x86Grab
OBJECTS = tcp_socket.o zynqGrabConfig.o zynqGrab.o

MAKEFLAGS = -j8

CFLAGS = -g3 -O0 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Wno-unused-but-set-variable

LDFLAGS = -pthread -L/usr/local/lib
LDLIBS = -lpthread

INC = -I../include

all: $(TARGET) fixed

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


ipAdd:
	@sudo ip address add 127.0.0.70/24 dev lo
	@sudo ip address add 127.0.0.71/24 dev lo
	@sudo ip address add 127.0.0.72/24 dev lo
	@sudo ip address add 127.0.0.73/24 dev lo

ipDel:
	@sudo ip address del 127.0.0.70/24 dev lo || true
	@sudo ip address del 127.0.0.71/24 dev lo || true
	@sudo ip address del 127.0.0.72/24 dev lo || true
	@sudo ip address del 127.0.0.73/24 dev lo || true

ipShow:
	@sudo ip address show lo

fixed: killFixed fixed0 fixed1 fixed2 fixed3

killFixed kill stop:
	@killall -q x86Grab || true

fixed0: zynqViewer $(TARGET)
	# @../../../tools/zynqViewer/zynqViewer -b -r -f ~/data/grab0_$(fileread).bin -o cam0Fixed.bin
	# @nice -n 15 ./$(TARGET) -i 127.0.0.1 -p 44444 -l 0 -f -n cam0Fixed.bin &
	@nice -n 15 ./$(TARGET) -i 127.0.0.1 -p 44444 -l 0 -f -n $(path)/grab0_$(fileread).bin &

fixed1: zynqViewer $(TARGET)
	# @../../../tools/zynqViewer/zynqViewer -b -r -f ~/data/grab1_$(fileread).bin -o cam1Fixed.bin
	# @nice -n 15 ./$(TARGET) -i 127.0.0.1 -p 44444 -l 1 -f -n cam1Fixed.bin &
	@nice -n 15 ./$(TARGET) -i 127.0.0.1 -p 44444 -l 1 -f -n $(path)/grab1_$(fileread).bin &

fixed2: zynqViewer $(TARGET)
	# @../../../tools/zynqViewer/zynqViewer -b -r -f ~/data/grab2_$(fileread).bin -o cam2Fixed.bin
	# @nice -n 15 ./$(TARGET) -i 127.0.0.1 -p 44444 -l 2 -f -n cam2Fixed.bin &
	@nice -n 15 ./$(TARGET) -i 127.0.0.1 -p 44444 -l 2 -f -n $(path)/grab2_$(fileread).bin &

fixed3: zynqViewer $(TARGET)
	# @../../../tools/zynqViewer/zynqViewer -b -r -f ~/data/grab3_$(fileread).bin -o cam3Fixed.bin
	# @nice -n 15 ./$(TARGET) -i 127.0.0.1 -p 44444 -l 3 -f -n cam3Fixed.bin &
	@nice -n 15 ./$(TARGET) -i 127.0.0.1 -p 44444 -l 3 -f -n $(path)/grab3_$(fileread).bin &

zynqViewer:
	@$(MAKE) --no-print-directory MAKEFLAGS= -C ../../../tools/zynqViewer viewer

killGrabClient: 
	@echo "## kill grabClient ##"
	@killall -q grabClient && sleep 2 || true

clean:
	@$(MAKE) --no-print-directory MAKEFLAGS= -C ../../../tools/zynqViewer clean
	@rm -f *.o *.bin $(TARGET)
	@sync

.PHONY: all clean fixed fixed0 fixed1 fixed2 fixed3 zynqViewer killGrabClient

.SECONDARY:
