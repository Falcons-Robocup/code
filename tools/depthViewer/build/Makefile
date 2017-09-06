# Copyright Andre Pool, Dec 2015
# Licensed under the Apache License version 2.0
# You may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

TARGET = depthViewer
OBJECTS = depthViewer.o

MAKEFLAGS = -j8

CFLAGS0 = -g3 -O2 -m64 -ansi -Wextra -Wpedantic -Wno-unused-but-set-variable -Wno-unused-parameter
CFLAGS = $(CFLAGS0) -Wall

LDFLAGS = -L/usr/local/lib 
LDLIBS = -rdynamic -lfreenect2 -lglfw -lGL -lusb-1.0 -lpthread -lglfw -lGL -lopencv_core -lopencv_imgproc -lopencv_highgui

INC = -I../include 

all: $(TARGET)
	@./$(TARGET)

viewer.o: ../src/viewer.cpp Makefile
	g++ $(CFLAGS0) $(INC) -o $@ -c $<

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	@sync
	@rm -f *.o $(TARGET)

.PHONY: all clean

.SECONDARY:
