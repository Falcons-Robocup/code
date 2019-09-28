# Copyright 2017 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = zynqViewer
OBJECTS = zynqViewer.o

MAKEFLAGS = -j8

# APOX add -Wextra
CFLAGS = -g3 -O0 -ansi -std=c++11 -Wextra -Wall -Wextra -Wpedantic -Wno-unused-but-set-variable

LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

all: $(TARGET)
	@./$(TARGET) -r -f ~/falcons/data/internal/vision/multiCam/grab0_2018-03-10_19:27:40.jpg -o zynqGrab.bin
	@./$(TARGET) -f zynqGrab.bin -o zynqGrab.jpg -c zynqGrabCrop.jpg
	qiv zynqGrab.jpg

jpg: $(TARGET)
	@./$(TARGET) -f ~/data/grab0_2018-04-29_13:41:14.bin -o zynqGrab.jpg
	qiv zynqGrab.jpg

%.o: %.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

viewer: $(TARGET)
	@: # trick to suppress Nothing to be done message

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


clean:
	@sync
	@rm -f *.o *.jpg *.bin $(TARGET)

.PHONY: all clean viewer jpg

.SECONDARY:
