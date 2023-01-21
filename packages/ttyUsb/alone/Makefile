# Copyright 2016 Andre Pool, February 2016
# Licensed under the Apache License version 2.0
# You may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

TARGET = ttyUsbAlone
OBJECTS = drv8301.o motorBoard.o serial.o ttyUsbAlone.o ttyUsb.o

MAKEFLAGS = -j8

CFLAGS0 = -g3 -O2 -m64 -ansi -Wextra -Wpedantic -Wno-unused-but-set-variable -Wno-unused-parameter
CFLAGS = $(CFLAGS0) -Wall -std=c++17

LDFLAGS = -L/usr/local/lib 
LDLIBS = -lpthread

INC = -I../include 

all: $(TARGET)
	@ stdbuf -i0 -o0 -e0 ./$(TARGET) | tee log

ble: $(TARGET)
	@ stdbuf -i0 -o0 -e0 ./$(TARGET) | tee log | grep BLE

bri: $(TARGET)
	@ stdbuf -i0 -o0 -e0 ./$(TARGET) | tee log | grep BRI

com: $(TARGET)
	@ stdbuf -i0 -o0 -e0 ./$(TARGET) | tee log | grep COM

cmps: $(TARGET)
	@ stdbuf -i0 -o0 -e0 ./$(TARGET) | tee log | grep CMPS

wre: $(TARGET)
	@ stdbuf -i0 -o0 -e0 ./$(TARGET) | tee log | grep WRE

wle: $(TARGET)
	@ stdbuf -i0 -o0 -e0 ./$(TARGET) | tee log | grep WLE

wri: $(TARGET)
	@ stdbuf -i0 -o0 -e0 ./$(TARGET) | tee log | grep WRI

pstree:
	@pstree -p `ps u | grep ttyUsbAlone | grep -v grep | awk '{print $$2}'`

ps:
	@ps -L -o pid,lwp,pri,nice,start,stat,bsdtime,cmd,comm -C ttyUsbAlone

num:
	@ps -L -o pid,lwp,pri,nice,start,stat,bsdtime,cmd,comm -C ttyUsbAlone | grep Alone | wc -l


%.o: ../src/%.cpp ../include/ttyUsb.hpp Makefile 
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	@sync
	@rm -f *.o $(TARGET)

.PHONY: all ble com cmps wre pstree ps num clean

.SECONDARY:
