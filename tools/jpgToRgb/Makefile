# Copyright 2014-2019 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = jpgToRgb
OBJECTS = $(TARGET).o

MAKEFLAGS = -j8

# Warning: using -O0 will not generate relevant warnings
CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror

LDFLAGS = -pthread -L/usr/local/lib
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lpthread

INC = -I./ -I/usr/include/opencv4

path=~/falcons/data/internal/vision/multiCam/r6
fileread = 20190427_224613

all: $(TARGET)
	@./$(TARGET) -v -d -i $(path)/grab0_$(fileread).jpg -o grab0.rgb
	@./$(TARGET) -v -d -i $(path)/grab1_$(fileread).jpg -o grab1.rgb
	@./$(TARGET) -v -d -i $(path)/grab2_$(fileread).jpg -o grab2.rgb
	@./$(TARGET) -v -d -i $(path)/grab3_$(fileread).jpg -o grab3.rgb

%.o: ./%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


clean:
	@sync
	@rm -f *.o $(TARGET) *.rgb

.PHONY: all clean

.SECONDARY:
