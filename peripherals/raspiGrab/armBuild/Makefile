# Copyright 2018 Andre Pool
# SPDX-License-Identifier: Apache-2.0

# location of raspi userland applications from https://github.com/raspberrypi/userland
# run buildme to generate the libraries
# unfortunately the build for arm1176 instead of Cortex A53 (64 bit)
# some files (e.g. kronos assembly) are incompatible with aarch64-linux-elf-gcc
# so for now stay with 32 bit arm1176 and move heavy stuff to separate optimized process
USERLAND_DIR=/home/robocup/tmp/userland

TARGET = raspiGrab
OBJECTS = RaspiCamControl.o RaspiCLI.o RaspiPreview.o RaspiVidYUVFalcons.o

MAKEFLAGS = -j8

# TODO: optimize flags and add -Wextra -Wpedantic
CFLAGS = -Wno-multichar -Wall -Wno-unused-but-set-variable -fPIC -O3 -DNDEBUG
CFLAGS += -mcpu=arm1176jzf-s -mfpu=vfp -mfloat-abi=hard -marm

LDFLAGS = -Wno-multichar -Wall -Wno-unused-but-set-variable -fPIC -O3 -DNDEBUG
LDLIBS = \
  $(USERLAND_DIR)/build/lib/libmmal_core.so \
  $(USERLAND_DIR)/build/lib/libmmal_util.so \
  $(USERLAND_DIR)/build/lib/libmmal_vc_client.so \
  $(USERLAND_DIR)/build/lib/libbcm_host.so \
  $(USERLAND_DIR)/build/lib/libvcsm.so \
  $(USERLAND_DIR)/build/lib/libvchostif.a \
  $(USERLAND_DIR)/build/lib/libvchiq_arm.so \
  $(USERLAND_DIR)/build/lib/libvcos.so -lpthread -ldl -lrt \
  $(USERLAND_DIR)/build/lib/libvcfiled_check.a

INC = -I../../../tools/raspiControl/include \
  -I $(USERLAND_DIR)/build/inc \
  -I $(USERLAND_DIR)/host_applications/framework \
  -I $(USERLAND_DIR) \
  -I $(USERLAND_DIR)/interface/vcos/pthreads \
  -I $(USERLAND_DIR)/interface/vmcs_host/linux \
  -I $(USERLAND_DIR)/interface/vmcs_host \
  -I $(USERLAND_DIR)/interface/vmcs_host/khronos \
  -I $(USERLAND_DIR)/interface/khronos/include \
  -I $(USERLAND_DIR)/build/arm-linux/release \
  -I $(USERLAND_DIR)/interface/vchiq_arm \
  -I $(USERLAND_DIR)/host_support/include \
  -I $(USERLAND_DIR)/interface/mmal \
  -I $(USERLAND_DIR)/host_applications/linux/libs/bcm_host/include \
  -I $(USERLAND_DIR)/host_applications/linux/apps/raspicam \
  -I $(USERLAND_DIR)/host_applications/linux/libs/sm

all: $(TARGET)
	ssh cam0 "rm -f raspiGrab/raspiGrab"
	scp raspiGrab cam0:raspiGrab/raspiGrab
	ssh cam0 "sudo killall raspiGrab"

# rule for cross compiling on x86-64
%.o: ../src/%.c Makefile ../../../tools/raspiControl/include/raspiDefaults.hpp
	arm-linux-gnueabihf-gcc $(CFLAGS) $(INC) -o $@ -c $<

# rule for building on the raspi
%.o: %.c raspiDefaults.hpp
	arm-linux-gnueabihf-gcc $(CFLAGS) $(INC) -o $@ -c $<
	cp $< ~/tmp

%.o: $(USERLAND_DIR)/host_applications/linux/apps/raspicam/%.c
	arm-linux-gnueabihf-gcc $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	arm-linux-gnueabihf-gcc $(LDFLAGS) -o $@ $^ -rdynamic $(LDLIBS) -Wl,-rpath,$(USERLAND_DIR)/build/lib

clean:
	@rm -f *.o $(TARGET)
	@sync

.PHONY: all clean

.SECONDARY:
