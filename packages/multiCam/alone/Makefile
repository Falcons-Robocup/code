# Copyright 2014-2022 Andre Pool
# SPDX-License-Identifier: Apache-2.0

TARGET = multiCamAlone
OBJECTS = BallDistanceEstimator.o ballDetection.o obstacleDetection.o cameraReceive.o camSysReceive.o dewarp.o \
configurator.o determinePosition.o fieldLut.o linePointDetection.o localization.o \
multicastSend.o preprocessor.o robotFloor.o simplex.o viewer.o multiCamAlone.o multiCam.o \
AbstractInterpolator.o cvmatio.o LeastSquaresInterpolator.o LinearInterpolator.o

MAKEFLAGS = -j8

# Warning: using -O0 will not generate relevant warnings
CFLAGS = -g3 -O3 -m64 -ansi -std=c++17 -Wall -Wextra -Wpedantic -Werror -DNOROS

LDFLAGS = -pthread -L/usr/local/lib -L${HOME}/falcons/code/build/packages/facilities/common
LDLIBS = -lalglib -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_videoio -lpthread -lboost_thread

INC = -I../include -I../../facilities/filters/inc -I../../facilities/filters/inc/ext -I/usr/include/opencv4

all: r6

r1: $(TARGET)
	@rm -f /dev/shm/cam0.jpg /dev/shm/cam1.jpg /dev/shm/cam2.jpg /dev/shm/cam3.jpg
	@nice -n 15 ./$(TARGET) -i 1

r2: $(TARGET)
	@rm -f /dev/shm/cam0.jpg /dev/shm/cam1.jpg /dev/shm/cam2.jpg /dev/shm/cam3.jpg
	@nice -n 15 ./$(TARGET) -i 2

r3: $(TARGET)
	@rm -f /dev/shm/cam0.jpg /dev/shm/cam1.jpg /dev/shm/cam2.jpg /dev/shm/cam3.jpg
	@nice -n 15 ./$(TARGET) -i 3

r4: $(TARGET)
	@rm -f /dev/shm/cam0.jpg /dev/shm/cam1.jpg /dev/shm/cam2.jpg /dev/shm/cam3.jpg
	@nice -n 15 ./$(TARGET) -i 4

r5: $(TARGET)
	@rm -f /dev/shm/cam0.jpg /dev/shm/cam1.jpg /dev/shm/cam2.jpg /dev/shm/cam3.jpg
	@nice -n 15 ./$(TARGET) -i 5

r6: $(TARGET)
	@rm -f /dev/shm/cam0.jpg /dev/shm/cam1.jpg /dev/shm/cam2.jpg /dev/shm/cam3.jpg
	@nice -n 15 ./$(TARGET) -i 6

r7: $(TARGET)
	@rm -f /dev/shm/cam0.jpg /dev/shm/cam1.jpg /dev/shm/cam2.jpg /dev/shm/cam3.jpg
	@nice -n 15 ./$(TARGET) -i 7

lo:
	@echo "###################################################################################################"
	@echo "## use local interface (lo) for the multicast data to prevent overloading the (wireless) network ##"
	@echo "###################################################################################################"
	@ip link show lo | grep -q MULTICAST || sudo ip link set dev lo multicast on
	@route -n | grep "224.16.16.0" | grep -q " eth0" && sudo route -n del -net 224.16.16.0 netmask 255.255.255.0 dev eth0 || true
	@route -n | grep "224.16.16.0" | grep -q " lo"   || sudo route -n add -net 224.16.16.0 netmask 255.255.255.0 dev lo

# use the following when no wired or wireless interface available
lo2:
	sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev lo
	sudo ifconfig lo multicast

wlan wlan0:
	sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlan0
	sudo ifconfig wlan0 multicast

wlp wlp2 wlp2s0:
	sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlp2s0
	sudo ifconfig wlp2s0 multicast

wlp3 wlp3s0:
	sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlp3s0
	sudo ifconfig wlp3s0 multicast

ETHERNET=`multiCamNet`
eth:
	@echo "## use $(ETHERNET) for the multicast data ##"
	@ip link show $(ETHERNET) | grep -q MULTICAST || sudo ip link set dev $(ETHERNET) multicast on
	@route | grep "224.16.16.0" | grep -q " lo" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 || true
	@route | grep "224.16.16.0" | grep -q " $(ETHERNET)" || sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev $(ETHERNET)

rdel:
	@echo "## use $(ETHERNET) for the multicast data ##"
	@route | grep "224.16.16.0" | grep -q " lo" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 || true
	@route | grep "224.16.16.0" | grep -q " $(ETHERNET)" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 dev $(ETHERNET) || true

demoField: $(TARGET)
	@./$(TARGET) -d dimensionsTechUnited -i 5

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../../facilities/filters/src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)


# show available camera's
list:
	@v4l2-ctl --list-devices


# show camera capabilities
cap:
	@v4l2-ctl -L --all || true

# show camera resolutions/fps
format:
	@v4l2-ctl --list-formats-ext


clean:
	@sync
	@rm -f *.o $(TARGET)
	@rm -f /dev/shm/cam0.jpg /dev/shm/cam1.jpg /dev/shm/cam2.jpg /dev/shm/cam3.jpg

.PHONY: all clean cap list format eth rdel lo r1 r2 r3 r4 r5 r6 r7

.SECONDARY:
