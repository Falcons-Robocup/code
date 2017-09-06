# Copyright 2015 Andre Pool
# Licensed under the Apache License version 2.0
# You may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

CFLAGS = -g -O2 -Wall -ansi -std=c99 -Wpedantic 
# next required for nanosleep
# CFLAGS += -D_POSIX_C_SOURCE=199309L

INC = -I. -I../inc

all: motorTest
	@./killall killMotorTest 2>/dev/null || true
	# @./killMotorTest &
	@./$< -P /dev/ttyUSB0

test: motorTest
	@./killall killMotorTest 2>/dev/null || true
	# @./killMotorTest &
	@./$< -P /dev/ttyUSB0 -a 7

%.o: %.c
	gcc $(CFLAGS) $(INC) -o $@ -c $<
	
motorTest: motorTest.o
	gcc $(CFLAGS) -o $@ $^ -lm

resetBoard: resetBoard.o
	gcc $(CFLAGS) -o $@ $^ -lm

reset: resetBoard
	@./resetBoard -q -P /dev/ttyUSB0

clean:
	@rm -f *.o motorTest resetBoard
	@sync

.PHONY: all clean reset

.SECONDARY:
