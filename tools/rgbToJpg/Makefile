# Copyright 2019 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = rgbToJpg
OBJECTS = $(TARGET).o

MAKEFLAGS = -j8

# Warning: using -O0 will not generate relevant warnings
CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror

LDFLAGS = -pthread -L/usr/local/lib
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

INC = -I./

path=~/falcons/data/internal/vision/multiCam/r6
timestamp = 20190428_135216

all: $(TARGET)
	@./$(TARGET) -d -v -i $(path)/grab0_$(timestamp).rgb -o grab0.jpg
	@./$(TARGET) -d -v -i $(path)/grab1_$(timestamp).rgb -o grab1.jpg
	@./$(TARGET) -d -v -i $(path)/grab2_$(timestamp).rgb -o grab2.jpg
	@./$(TARGET) -d -v -i $(path)/grab3_$(timestamp).rgb -o grab3.jpg

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


clean:
	@sync
	@rm -f *.o $(TARGET) *.jpg

.PHONY: all clean

.SECONDARY:
