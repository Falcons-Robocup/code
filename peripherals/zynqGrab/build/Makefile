# Copyright 2017 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = zynqGrab

filedate = $(shell date +%Y-%m-%d_%H:%M:%S)
# fileread = 2018-03-10_19:27:51
# fileread = 2018-03-10_19:27:40
# fileread = 2018-03-10_19:25:30
# fileread = 2018-03-10_19:33:54
# fileread = 2018-03-10_19:37:01
# fileread = 2018-03-22_20:43:21
# fileread = 2018-03-22_20:11:17
# fileread = 2018-03-22_20:00:56
# fileread = 2018-04-29_13:41:14
# fileread = 2018-04-29_13:33:21
# fileread = 2018-04-29_13:35:52
fileread = 2018-04-29_13:37:43



# from Vivado compilation bare metal
# Building file: ../src/helloworld.c
# arm-none-eabi-gcc -Wall -O0 -g3 -c -fmessage-length=0 -MT"src/helloworld.o" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -I../../andre_application_project_bsp/ps7_cortexa9_0/include -MMD -MP -MF"src/helloworld.d" -MT"src/helloworld.o" -o "src/helloworld.o" "../src/helloworld.c"
# Building file: ../src/platform.c
# arm-none-eabi-gcc -Wall -O0 -g3 -c -fmessage-length=0 -MT"src/platform.o" -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -I../../andre_application_project_bsp/ps7_cortexa9_0/include -MMD -MP -MF"src/platform.d" -MT"src/platform.o" -o "src/platform.o" "../src/platform.c"
# Linker = Building target: andre_application_project.elf
# arm-none-eabi-gcc -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -Wl,-build-id=none -specs=Xilinx.spec -Wl,-T -Wl,../src/lscript.ld -L../../andre_application_project_bsp/ps7_cortexa9_0/lib -o "andre_application_project.elf"  ./src/helloworld.o ./src/platform.o   -Wl,--start-group,-lxil,-lgcc,-lc,--end-group
 
# Invoking: ARM v7 Print Size
# arm-none-eabi-size andre_application_project.elf  |tee "andre_application_project.elf.size"
#    text	   data	    bss	    dec	    hex	filename
#   22328	   1148	  22568	  46044	   b3dc	andre_application_project.elf

MAKEFLAGS = -j8

CC = arm-linux-gnueabihf-g++

CFLAGS = -g3 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Wno-unused-but-set-variable
# Zynq contains Cortex A9 which uses armv7-a instructions set
CFLAGS += -Ofast -fno-common -ffast-math -ftree-vectorize -march=armv7-a -mtune=cortex-a9 -mthumb -mvectorize-with-neon-quad -mfloat-abi=hard
# neon is faster then vfpv3 for integer
CFLAGS += -mfpu=neon
# CFLAGS += -mfpu=vfpv3

INC = -I../include
# -I/opt/opencv_arm/include

LDFLAGS = -pthread
LDLIBS = -lpthread

all: zynq
%.o: ../src/%.cpp Makefile
	$(CC) $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(TARGET).o tcp_socket.o $(TARGET)Config.o
	$(CC) $(LDFLAGS) -o $@ $^ $(LDLIBS)
	@file $(TARGET)

zynq: cam0 cam1 cam2 cam3

cam0: stop0 $(TARGET)
	@scp $(TARGET) cam0:/data
	@ssh cam0 "sync"
	@ssh -t cam0 "/data/zynqGrab -l 0 -i 10.0.0.65 -p 44444 || true" &
#	@ #scp zynq:$(TARGET).png .
#	@scp zynq:$(TARGET).bin .
#	@~/zynqberry/andre/framebuffer/viewer
#	@qiv $(TARGET).png

cam1: stop1 $(TARGET)
	@scp $(TARGET) cam1:/data
	@ssh cam1 "sync"
	@ssh -t cam1 "rm -f $(TARGET).bin; /data/zynqStartup || true" &

cam2: stop2 $(TARGET)
	@scp $(TARGET) cam2:/data
	@ssh cam2 "sync"
	@ssh -t cam2 "rm -f $(TARGET).bin; /data/zynqStartup || true" &

cam3: stop3 $(TARGET)
	@scp $(TARGET) cam3:/data
	@ssh cam3 "sync"
	@ssh -t cam3 "rm -f $(TARGET).bin; /data/zynqStartup || true" &

grab: ../../../tools/zynqViewer/zynqViewer grab0 grab1 grab2 grab3

../../../tools/zynqViewer/zynqViewer:
	@$(MAKE) --no-print-directory MAKEFLAGS= -C ../../../tools/zynqViewer

grab0: stop0 $(TARGET)
	@scp $(TARGET) cam0:/data
	@ssh -t cam0 "rm -f $@.bin; /data/$(TARGET) -g || true"
	@scp cam0:zynqGrab.bin $@.bin
	@../../../tools/zynqViewer/zynqViewer -f $@.bin -o $@_$(filedate).jpg -c $@_$(filedate)_crop.jpg
	@cp $@.bin $@_$(filedate).bin
	@qiv $@_$(filedate).jpg &

grab1: stop1 $(TARGET)
	@scp $(TARGET) cam1:/data
	@ssh -t cam1 "rm -f $@.bin; /data/$(TARGET) -g || true"
	@scp cam1:zynqGrab.bin $@.bin
	@../../../tools/zynqViewer/zynqViewer -f $@.bin -o $@_$(filedate).jpg -c $@_$(filedate)_crop.jpg
	@cp $@.bin $@_$(filedate).bin
	@qiv $@_$(filedate).jpg

grab2: stop2 $(TARGET)
	@scp $(TARGET) cam2:/data
	@ssh -t cam2 "rm -f $@.bin; /data/$(TARGET) -g || true"
	@scp cam2:zynqGrab.bin $@.bin
	@../../../tools/zynqViewer/zynqViewer -f $@.bin -o $@_$(filedate).jpg -c $@_$(filedate)_crop.jpg
	@cp $@.bin $@_$(filedate).bin
	@qiv $@_$(filedate).jpg

grab3: stop3 $(TARGET)
	@scp $(TARGET) cam3:/data
	@ssh -t cam3 "rm -f $@.bin; /data/$(TARGET) -g || true"
	@scp cam3:zynqGrab.bin $@.bin
	@../../../tools/zynqViewer/zynqViewer -f $@.bin -o $@_$(filedate).jpg -c $@_$(filedate)_crop.jpg
	@cp $@.bin $@_$(filedate).bin
	@qiv $@_$(filedate).jpg

fixed: fixed0 fixed1 fixed2 fixed3

fixed0: stop0 zynqViewer $(TARGET)
	@scp $(TARGET) cam0:/data
	# @../../../tools/zynqViewer/zynqViewer -b -r -f ~/data/grab0_$(fileread).bin -o $@.bin
	@scp ~/data/grab0_$(fileread).bin cam0:zynqfixed.bin
	@ssh -t cam0 "/data/zynqGrab -l 0 -i 10.0.0.65 -p 44444 -f -n ~/zynqfixed.bin || true" &

fixed1: stop1 zynqViewer $(TARGET)
	@scp $(TARGET) cam1:/data
	# @../../../tools/zynqViewer/zynqViewer -b -r -f ~/data/grab1_$(fileread).bin -o $@.bin
	@scp ~/data/grab1_$(fileread).bin cam1:zynqfixed.bin
	@ssh -t cam1 "/data/zynqGrab -l 1 -i 10.0.0.65 -p 44444 -f -n ~/zynqfixed.bin || true" &

fixed2: stop2 zynqViewer $(TARGET)
	@scp $(TARGET) cam2:/data
	# @../../../tools/zynqViewer/zynqViewer -b -r -f ~/data/grab2_$(fileread).bin -o $@.bin
	@scp ~/data/grab2_$(fileread).bin cam2:zynqfixed.bin
	@ssh -t cam2 "/data/zynqGrab -l 2 -i 10.0.0.65 -p 44444 -f -n ~/zynqfixed.bin || true" &

fixed3: stop3 zynqViewer $(TARGET)
	@scp $(TARGET) cam3:/data
	# @../../../tools/zynqViewer/zynqViewer -b -r -f ~/data/grab3_$(fileread).bin -o $@.bin
	@scp ~/data/grab3_$(fileread).bin cam3:zynqfixed.bin
	@ssh -t cam3 "/data/zynqGrab -l 3 -i 10.0.0.65 -p 44444 -f -n ~/zynqfixed.bin || true" &

zynqViewer:
	@$(MAKE) --no-print-directory MAKEFLAGS= -C ../../../tools/zynqViewer viewer

killGrabClient: 
	@echo "## kill grabClient ##"
	@killall -q grabClient && sleep 2 || true

stop: stop0 stop1 stop2 stop3

stop0: killGrabClient ssh0
	@ssh cam0 "killall -q zynqRestarter bash $(TARGET) 2>/dev/null" || true

stop1: killGrabClient ssh1
	@ssh cam1 "killall -q zynqRestarter bash $(TARGET) 2>/dev/null" || true

stop2: killGrabClient ssh2
	@ssh cam2 "killall -q zynqRestarter bash $(TARGET) 2>/dev/null" || true

stop3: killGrabClient ssh3
	@ssh cam3 "killall -q zynqRestarter bash $(TARGET) 2>/dev/null" || true

ssh0:
	@cam0 true
ssh1:
	@cam1 true
ssh2:
	@cam2 true
ssh3:
	@cam3 true

restart reset reboot: ssh0 ssh1 ssh2 ssh3
	@ssh cam0 /sbin/reboot
	@ssh cam1 /sbin/reboot
	@ssh cam2 /sbin/reboot
	@ssh cam3 /sbin/reboot

1: ssh0 ssh1 ssh2 ssh3
	@ssh cam0 "echo /data/zynqGrab -l 0 -i 10.0.0.1 -p 44444 \$$\@ > /data/zynqStartup"
	@ssh cam1 "echo /data/zynqGrab -l 1 -i 10.0.0.1 -p 44444 \$$\@ > /data/zynqStartup"
	@ssh cam2 "echo /data/zynqGrab -l 2 -i 10.0.0.1 -p 44444 \$$\@ > /data/zynqStartup"
	@ssh cam3 "echo /data/zynqGrab -l 3 -i 10.0.0.1 -p 44444 \$$\@ > /data/zynqStartup"

65: ssh0 ssh1 ssh2 ssh3
	@ssh cam0 "echo /data/zynqGrab -l 0 -i 10.0.0.65 -p 44444 \$$\@ > /data/zynqStartup"
	@ssh cam1 "echo /data/zynqGrab -l 1 -i 10.0.0.65 -p 44444 \$$\@ > /data/zynqStartup"
	@ssh cam2 "echo /data/zynqGrab -l 2 -i 10.0.0.65 -p 44444 \$$\@ > /data/zynqStartup"
	@ssh cam3 "echo /data/zynqGrab -l 3 -i 10.0.0.65 -p 44444 \$$\@ > /data/zynqStartup"

netstat:
	@echo "cam0 : netstat -tuan"
	@ssh cam0 "netstat -tuan | grep TIME_WAIT || true"
	@echo "cam1 : netstat -tuan"
	@ssh cam1 "netstat -tuan | grep TIME_WAIT || true"
	@echo "cam2 : netstat -tuan"
	@ssh cam2 "netstat -tuan | grep TIME_WAIT || true"
	@echo "cam3 : netstat -tuan"
	@ssh cam3 "netstat -tuan | grep TIME_WAIT || true"


# select between the library of ubuntu or from the Xilinx tooling
lib: xillib

ubuntu:
	@scp /usr/arm-linux-gnueabihf/lib/libstdc++.so.6 zynq:/usr/lib
#	@scp /opt/opencv_arm/lib/libopencv_core.so.3.3 zynq:/usr/lib
#	@scp /opt/opencv_arm/lib/libopencv_imgproc.so.3.3 zynq:/usr/lib
#	@scp /opt/opencv_arm/lib/libopencv_imgcodecs.so.3.3 zynq:/usr/lib
#	@scp /opt/opencv_arm/lib/libopencv_videoio.so.3.3 zynq:/usr/lib


xillib:
	@scp /opt/Xilinx/SDK/2017.1/gnu/aarch32/lin/gcc-arm-linux-gnueabi/arm-linux-gnueabihf/lib/libstdc++.so.6 zynq:/usr/lib
#	@scp /opt/Xilinx/SDK/2017.4/gnu/aarch32/lin/gcc-arm-linux-gnueabi/arm-linux-gnueabihf/lib/libstdc++.so.6 zynq:/usr/lib

rmlib:
	@ssh zynq "rm -f /usr/lib/libstdc++.so.6"
	@ssh zynq "rm -f /usr/lib/libopencv_core.so.3.3"
	@ssh zynq "rm -f /usr/lib/libopencv_imgproc.so.3.3"
	@ssh zynq "rm -f /usr/lib/libopencv_imgcodecs.so.3.3"
	@ssh zynq "rm -f /usr/lib/libopencv_videoio.so.3.3"
	
clean:
	@$(MAKE) --no-print-directory MAKEFLAGS= -C ../../../tools/zynqViewer clean
	@rm -f *.o $(TARGET) $(TARGET).bin $(TARGET).jpg
	@sync

.PHONY: all clean zynq lib ubuntu xillib rmlib cam0 cam1 cam2 cam3 grab grab0 grab1 grab2 grab3 fixed fixed0 fixed1 fixed2 fixed3 zynqViewer stop stop0 stop1 stop2 stop3 ssh0 ssh1 ssh2 ssh3 netstat 1 65 reboot killGrabClient

.SECONDARY:
