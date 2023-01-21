# Copyright 2022 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = pylonToOpenCv
OBJECTS = grabber.o pylonToOpenCv.o

MAKEFLAGS = -j8

CFLAGS  = -O3 # # using -O0 will not generate relevant warnings
CFLAGS += -march=native -ansi -Wall -Wextra -Wpedantic -Werror
# CFLAGS += -std=c++17 # c++17 is same as c++1z
CFLAGS += -std=c++2a # c++2a will become c++20 in gcc version 10
CFLAGS += -Wno-deprecated-enum-enum-conversion # default c++20 does not allow arithmetic between different enumeration types, which are in openCV
CFLAGS += -pipe # speedup compilation process

LDFLAGS  = -Wl,--enable-new-dtags # create new dynamic tags in ELF
LDFLAGS += -Wl,-rpath,/opt/pylon/lib # add the pylon library path to the runtime library search path
# LDFLAGS += -L/usr/local/lib
LDFLAGS += -Wl,--export-dynamic # add all symbols to the dynamic symbol table
LDFLAGS += -pthread

LDLIBS  = -L/opt/pylon/lib # tell the linker where it can find the pylon libraries
LDLIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs  -lopencv_videoio
LDLIBS += -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon
LDLIBS += -lpthread

INC  = -I/opt/pylon/include
INC += -I/usr/include/opencv4

all: $(TARGET)
	@./$(TARGET)

0: $(TARGET)
	@./$(TARGET) -i 0

1: $(TARGET)
	@./$(TARGET) -i 1

2: $(TARGET)
	@./$(TARGET) -i 2

3: $(TARGET)
	@./$(TARGET) -i 3


%.o: %.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	@rm -f *.o $(TARGET)

.PHONY: 0 1 2 3 all clean

.SECONDARY:
