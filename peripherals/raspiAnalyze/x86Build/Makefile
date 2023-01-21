# Copyright 2018-2019 Andre Pool
# SPDX-License-Identifier: Apache-2.0

path=~/falcons/data/internal/vision/multiCam

# issue 107
# robot = r6
# date = 20200818
# time = 205032

# robot = r6
# date = 20200820
# time = 210857

robot = r5
date = 20220712
time = 172633

# path=~/data
# path=~/falcons/data/internal/vision/multiCam

# select the set of test images
# fileread = 2018-05-21_11:38:26
# fileread = 2018-05-21_18:03:32
# fileread = 2018-05-21_18:02:43
# fileread = 2018-05-24_20:43:01

# JFEI: I noticed far away obstacles to be accepted, but then projected close by -> ghost obstacles
# see grab 20180530_190342, in particular the obstacle at cam0
# fileread = 20180530_190342

# fileread = r2_assyI_20180604_211944
# fileread = r3_assyD_20180604_214511
# fileread = r4_20180603_162900
# fileread = r5_assyE_20180604_204542
# fileread = r6_assyH_20180605_193729
# fileread = 20180617_203549
# fileread = 20180618_005039
# fileread = 20180618_144451
# fileread = 20180618_165850
# fileread = 20180618_205930
# fileread = 20180618_005039
# fileread = 20180618_165850
# fileread = 20180618_235237
# fileread = 20180618_234631
# fileread = 20180619_214531
# fileread = 20180620_145644

# robot = r3
# date = 20180620
# time = 145644

# robot = r2
# date = 20180619
# time = 214554

# Montreal test field
# robot = r3
# date = 20180620
# time = 201643

# robot = r5
# date = 20180809
# time = 231123

# lots of balls on lines
# robot = r2
# date = 20180809
# time = 223758
# time = 223759
# time = 223800

# lots of balls on lines
# robot = r3
# date = 20180809
# time = 221830
# time = 221832
# time = 221833

# lots of balls on lines
# robot = r5
# date = 20180809
# time = 231123

# robot = r1
# date = 20181030
# time = 212155

# robot = r4
# date = 20180927
# time = 202435

# robot = r3
# date = 20190113
# time = 152336

# robot = r5
# date = 20190113
# time = 154010
# the real bad one
# date = 20190117
# time = 204745
# time = 205319
# time = 205333
# time = 205348

# robot = r3
# date = 20190220
# time = 001102

# grabs from Jan to demonstrate the Robot tilt
# robot = r5
# date = 20190217
# very good: score 0.001 - 0.006
# time = 161726

# very good: score 0.000 - 0.001
# time = 161658

# good: score 0.010 - 0.017
# time = 162111

# very good: score 0.000 - 0.012
# time = 162036

# good: score 0.004 - 0.024
# time = 162012

# good: score 0.006-0.017
# time = 161948

# acceptable : score 0.009-0.047
# time = 161815

# very good : score 0.000 - 0.002
# time = 161749

# new keeperframe
# robot = r1
# date = 20190223
# time = 135008

# robot = r7
# date = 20190402
# time = 185039

# robot = r2
# date = 20190404
# time = 224958

# ## porto 2019 ##
# robot = r6
# date = 20190426
# time = 191317
# time = 191806
# time = 191818
# time = 191759

# techunited robot 3 + eric (false balls)
# robot = r1
# date = 20190427
# time = 160053

# de locht against tech united
# path=~/falcons/data/internal/vision/multiCam/r4
# good examples for wrong obstacle detections
# robot = r4
# date = 20190606
# time = 212129
# time = 212209
# ball before robot
# time = 212312
# time = 212310
# time = 211932
# time = 211836
# time = 211839

# ## Sydney ##
# robot = r2
# date = 20190702
# time = 062150
# date = 20190703
# time = 051051
# date = 20190706
# time = 061811

# ## demo field for PSV demo, the plastic floor requires
# a modified floor color filter
# robot = r4
# date = 20191130
# time = 150800
# time = 163732
# time = 163555
# time = 163450

TARGET = raspiAnalyze
OBJECTS = $(TARGET).o

RASPI_SYSTEM_PATH = ../../../peripherals/raspiSystem/x86Build
RASPI_SYSTEM = $(RASPI_SYSTEM_PATH)/raspiSystem
JPG_TO_RGB_PATH = ../../../tools/jpgToRgb
JPG_TO_RGB = $(JPG_TO_RGB_PATH)/jpgToRgb

MAKEFLAGS = -j8

CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror

LDLIBS = -lpthread

INC = -I../include -I../../../packages/raspiControl/include

all: lo info $(TARGET) fixed

lo:
	@echo "###################################################################################################"
	@echo "## use local interface (lo) for the multicast data to prevent overloading the (wireless) network ##"
	@echo "###################################################################################################"
	@ip link show lo | grep -q MULTICAST || sudo ip link set dev lo multicast on
	@route -n | grep "224.16.16.0" | grep -q " eth0" && sudo route -n del -net 224.16.16.0 netmask 255.255.255.0 dev eth0 || true
	@route -n | grep "224.16.16.0" | grep -q " lo"   || sudo route -n add -net 224.16.16.0 netmask 255.255.255.0 dev lo

eth eth0:
	@echo "## use wired interface (eth0) for the multicast data to prevent overloading the (wireless) network ##"
	@ip link show eth0 | grep -q MULTICAST || sudo ip link set dev eth0 multicast on
	@route -n | grep "224.16.16.0" | grep -q " lo"   && sudo route -n del -net 224.16.16.0 netmask 255.255.255.0 dev lo || true
	@route -n | grep "224.16.16.0" | grep -q " eth0" || sudo route -n add -net 224.16.16.0 netmask 255.255.255.0 dev eth0

enx:
	@echo "## use wired interface (enx000acd2d1ae8) for the multicast data to prevent overloading the (wireless) network ##"
	@ip link show enx000acd2d1ae8 | grep -q MULTICAST || sudo ip link set dev enx000acd2d1ae8 multicast on
	@route -n | grep "224.16.16.0" | grep -q " lo"   && sudo route -n del -net 224.16.16.0 netmask 255.255.255.0 dev lo || true
	@route -n | grep "224.16.16.0" | grep -q " eth0"   && sudo route -n del -net 224.16.16.0 netmask 255.255.255.0 dev eth0 || true
	@route -n | grep "224.16.16.0" | grep -q " enx000acd2d1ae8" || sudo route -n add -net 224.16.16.0 netmask 255.255.255.0 dev enx000acd2d1ae8

info:
	@echo 
	@echo "## type \"make stop\" to stop the camera processes ##"
	@echo 

%.o: ../src/%.c Makefile ../../../packages/raspiControl/include/raspiDefaults.hpp 
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	@killall -q raspiAnalyze || true
	@killall -q raspiSystem || true
	g++ -o $@ $^ $(LDLIBS)


ipAdd:
	@echo "## these Ethernet addresses are not required anymore ##
	@sudo ip address add 127.0.0.100/24 dev lo
	@sudo ip address add 127.0.0.101/24 dev lo
	@sudo ip address add 127.0.0.102/24 dev lo
	@sudo ip address add 127.0.0.103/24 dev lo

ipDel:
	@sudo ip address del 127.0.0.100/24 dev lo || true
	@sudo ip address del 127.0.0.101/24 dev lo || true
	@sudo ip address del 127.0.0.102/24 dev lo || true
	@sudo ip address del 127.0.0.103/24 dev lo || true

ipShow:
	@sudo ip address show lo

fixed: fixed0 fixed1 fixed2 fixed3

killFixed kill stop:
	@echo killall running analyzers
	@killall -q raspiAnalyze || true
	@killall -q -9 raspiAnalyze || true
	@killall -q -15 raspiAnalyze || true
	@killall -q raspiSystem || true
	@killall -q -9 raspiSystem || true
	@killall -q -15 raspiSystem || true

$(RASPI_SYSTEM):
	@$(MAKE) --no-print-directory -C $(RASPI_SYSTEM_PATH) raspiSystem

fixed0: $(TARGET) $(RASPI_SYSTEM) cam0Image.rgb
	@make --no-print-directory -C $(RASPI_SYSTEM_PATH) r0 || true &
	@nice -n 15 ./$(TARGET) -l 0 &

fixed1: $(TARGET) $(RASPI_SYSTEM) cam1Image.rgb
	@make --no-print-directory -C $(RASPI_SYSTEM_PATH) r1 || true &
	@nice -n 15 ./$(TARGET) -l 1 &

fixed2: $(TARGET) $(RASPI_SYSTEM) cam2Image.rgb
	@make --no-print-directory -C $(RASPI_SYSTEM_PATH) r2 || true &
	@nice -n 15 ./$(TARGET) -l 2 &

fixed3: $(TARGET) $(RASPI_SYSTEM) cam3Image.rgb
	@make --no-print-directory -C $(RASPI_SYSTEM_PATH) r3 || true &
	@nice -n 15 ./$(TARGET) -l 3 &

valgrind: $(TARGET) cam0Image.rgb
	valgrind --tool=memcheck --leak-check=full --show-reachable=yes \
	--num-callers=20 --track-fds=yes --track-origins=yes \
	./$(TARGET) -l 0 -t 6

valgrind100: $(TARGET) cam0Image.rgb
	valgrind --tool=memcheck --leak-check=full --show-reachable=yes \
	--num-callers=20 --track-fds=yes --track-origins=yes \
	./$(TARGET) -l 0 -t 100

rgb: cam0Image.rgb cam1Image.rgb cam2Image.rgb cam3Image.rgb

$(JPG_TO_RGB):
	@$(MAKE) --no-print-directory -C $(JPG_TO_RGB_PATH) jpgToRgb

# center spot : 115144
# GRAB = 20190428_115144

# techunited close by
# GRAB = 20190428_115733

# techunited ball on orange cover !!!!!
# GRAB = 20190428_114659

cam0Image.rgb: $(JPG_TO_RGB) Makefile
	@$(JPG_TO_RGB) -i $(path)/$(robot)/$(date)/cam0_$(date)_$(time).jpg -o cam0Image.rgb
#	@cp ~/falcons/data/internal/vision/multiCam/r6/grab0_$(GRAB).rgb cam0Image.rgb
cam1Image.rgb: $(JPG_TO_RGB) Makefile
	@$(JPG_TO_RGB) -i $(path)/$(robot)/$(date)/cam1_$(date)_$(time).jpg -o cam1Image.rgb
#	@cp ~/falcons/data/internal/vision/multiCam/r6/grab1_$(GRAB).rgb cam1Image.rgb
cam2Image.rgb: $(JPG_TO_RGB) Makefile
	@$(JPG_TO_RGB) -i $(path)/$(robot)/$(date)/cam2_$(date)_$(time).jpg -o cam2Image.rgb
#	@cp ~/falcons/data/internal/vision/multiCam/r6/grab2_$(GRAB).rgb cam2Image.rgb
cam3Image.rgb: $(JPG_TO_RGB) Makefile
	@$(JPG_TO_RGB) -i $(path)/$(robot)/$(date)/cam3_$(date)_$(time).jpg -o cam3Image.rgb
#	@cp ~/falcons/data/internal/vision/multiCam/r6/grab3_$(GRAB).rgb cam3Image.rgb

killGrabClient: 
	@echo "## kill grabClient ##"
	@killall -q grabClient && sleep 2 || true

clean: killFixed
	@rm -f *.o *.bin *.rgb $(TARGET)
	@$(MAKE) --no-print-directory -C $(RASPI_SYSTEM_PATH) clean
	@$(MAKE) --no-print-directory -C $(JPG_TO_RGB_PATH) clean
	@sync

.PHONY: all clean fixed fixed0 fixed1 fixed2 fixed3 killGrabClient killFixed ipAdd ipDel ipShow rgb info lo eth eth0 enx valgrind valgrind100

.SECONDARY:
