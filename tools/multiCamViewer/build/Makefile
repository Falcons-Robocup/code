# Copyright 2014-2018 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = multiCamViewer
OBJECTS = camSysReceive.o $(TARGET).o

MAKEFLAGS = -j8

CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror

LDFLAGS = -pthread -L/usr/local/lib
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lpthread

INC = -I../include -I../../../packages/multiCam/include/ -I/usr/include/opencv4

all: eth $(TARGET)
	@nice -n 15 ./$(TARGET)

0: eth $(TARGET)
	@nice -n 15 ./$(TARGET) -0

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../../../packages/multiCam/src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


ETHERNET=`multiCamNet`
eth:
	@echo "## use $(ETHERNET) for the multicast data ##"
	@ip link show $(ETHERNET) | grep -q MULTICAST || sudo ip link set dev $(ETHERNET) multicast on
	@route | grep "224.16.16.0" | grep -q " lo" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 || true
	@route | grep "224.16.16.0" | grep -q " $(ETHERNET)" || sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev $(ETHERNET)

size:
	@echo "maximal UDP receive buffer size: `cat /proc/sys/net/core/rmem_max` bytes"
	@echo "default UDP receive buffer size: `cat /proc/sys/net/core/rmem_default` bytes"

set:
	# on systems with 16GiB memory the max is 50000000 bytes (50Mbytes) and default is 212992 bytes (208KiB)
	# camera maximal UDP packet is almost 64KiB
	# when camera is running at 30FPS and every frame 4 packets of the image are send
	# then with 2 camera's there is about 1 drop every 30 seconds when the a 
	# default and max buffer size of 512KiB (=524288)
	# so for 4 camemera's the double amount should be sufficient
	@sudo sysctl -w net.core.rmem_default=524288
	# @sudo sysctl -w net.core.rmem_max=524288

set64:
	# 64KiB (=65536)
	@sudo sysctl -w net.core.rmem_default=65536

set128:
	# 128KiB (=131072)
	@sudo sysctl -w net.core.rmem_default=131072


set256:
	# 256KiB (=262144)
	@sudo sysctl -w net.core.rmem_default=262144

set512:
	# 512KiB (=524288)
	@sudo sysctl -w net.core.rmem_default=524288
 
set1:
	# 1MiB (=1048576)
	@sudo sysctl -w net.core.rmem_default=1048576

set2:
	# 2MiB (=2097152)
	@sudo sysctl -w net.core.rmem_default=2097152

set4:
	# 4MiB (=4194304)
	@sudo sysctl -w net.core.rmem_default=4194304

set8:
	# 4MiB (=8388608)
	@sudo sysctl -w net.core.rmem_default=8388608

dropped:
	@netstat -suna

dump:
	@sudo tcpdump -ni eth0

clean:
	@sync
	@rm -f *.o $(TARGET)
	@rm -f *.jpg

.PHONY: all size set set1 dropped clean ana eth

.SECONDARY:
