# Copyright 2018-2021 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = jetsonAnalyze
OBJECTS = $(TARGET).o

MAKEFLAGS = -j8

CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror

LDLIBS = -lpthread -lrt

INC = -I../../../packages/raspiControl/include

all: $(TARGET)
	@./$(TARGET)

%.o: ../src/%.c Makefile ../../../packages/raspiControl/include/raspiDefaults.hpp
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ -o $@ $^ $(LDLIBS)

valgrind: $(TARGET)
	valgrind --tool=memcheck --leak-check=full --show-reachable=yes \
	--num-callers=20 --track-fds=yes --track-origins=yes \
	./$(TARGET) -l 0 -t 6

valgrind100: $(TARGET)
	valgrind --tool=memcheck --leak-check=full --show-reachable=yes \
	--num-callers=20 --track-fds=yes --track-origins=yes \
	./$(TARGET) -l 0 -t 100

clean:
	@rm -f *.o $(TARGET)

.PHONY: all clean valgrind valgrind100

.SECONDARY:
