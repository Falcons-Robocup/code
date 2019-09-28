# Copyright 2018 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = raspiSystem
OBJECTS = $(TARGET).o raspiRgbToBmp.o

MAKEFLAGS = -j8

# use gnu11 instead of c11 to prevent storage size of 'mreq' isn't known
CFLAGS = -Ofast -ansi -std=gnu11 -Wno-multichar -Wall -Wno-unused-but-set-variable -Wextra -Wpedantic
CFLAGS += -fno-common
CFLAGS += -ffast-math
CFLAGS += -ftree-vectorize
CFLAGS += -marm 
CFLAGS += -mfpu=vfp
CFLAGS += -mfloat-abi=hard
CFLAGS += -mabi=aapcs-linux
CFLAGS += -march=armv8-a+crc
CFLAGS += -mfloat-abi=hard
CFLAGS += -mfpu=neon-fp-armv8 # other options are 'fp-armv8', 'neon-fp-armv8', and 'crypto-neon-fp-armv8'
CFLAGS += -funsafe-math-optimizations
CFLAGS += -mhard-float
CFLAGS += -mlittle-endian
CFLAGS += -mtls-dialect=gnu2 # or just gnu if gnu2 doesn't work
CFLAGS += -mtune=cortex-a53
CFLAGS += -munaligned-access
CFLAGS += -mvectorize-with-neon-quad

LDFLAGS = 
LDLIBS = -lpthread -lrt

INC = -I../../../packages/raspiControl/include

all: $(TARGET)
	@file $(TARGET)

# rule for building on the raspi
%.o: %.c raspiDefaults.hpp
	arm-linux-gnueabihf-gcc $(CFLAGS) $(INC) -o $@ -c $<
#	cp $< ~/tmp

%.o: ../src/%.c Makefile ../../../packages/raspiControl/include/raspiDefaults.hpp
	arm-linux-gnueabihf-gcc $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	arm-linux-gnueabihf-gcc $(LDFLAGS) -o $@ $^ $(LDLIBS)

clean:
	@rm -f *.o $(TARGET)
	@sync

.PHONY: all clean

.SECONDARY:
