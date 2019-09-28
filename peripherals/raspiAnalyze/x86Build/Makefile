# Copyright 2018-2019 Andre Pool
# SPDX-License-Identifier: Apache-2.0

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

# path=~/falcons/data/internal/vision/multiCam/r2_ball
# fileread = 20180619_214554


# path=~/falcons/data/internal/vision/multiCam/r3_test_field
# fileread = 20180620_201643


# path=~/falcons/data/internal/vision/multiCam/r2
# fileread = 20180809_223758
# fileread = 20180809_223759
# fileread = 20180809_223800


# path=~/falcons/data/internal/vision/multiCam/r3
# fileread = 20180809_221830
# fileread = 20180809_221832
# fileread = 20180809_221833

# path=~/falcons/data/internal/vision/multiCam/r5
# fileread = 20180809_231123


# path=~/falcons/data/internal/vision/multiCam
# fileread = r1_assyG_20181030_212155
# fileread = r4_assyF_20180927_202435

# path=~/falcons/data/internal/vision/multiCam/r3
# fileread = 20190113_152336

# path=~/falcons/data/internal/vision/multiCam/r1
# fileread = 20190113_154010
# the real bad one
# fileread = 20190117_204745
# fileread = 20190117_205319
# fileread = 20190117_205333
# fileread = 20190117_205348
# fileread = 20190220_001102
# fileread = 20190223_135008

# new grabs from Jan to demonstrate the Robot tilt
# very good: scroe 0.001 - 0.006
# fileread = 20190217_161726

# very good: score 0.000 - 0.001
# fileread = 20190217_161658

# good: score 0.010 - 0.017
# fileread = 20190217_162111

# very good: score 0.000 - 0.012
# fileread = 20190217_162036

# good: score 0.004 - 0.024
# fileread = 20190217_162012

# good: score 0.006-0.017
# fileread = 20190217_161948

# acceptable : score 0.009-0.047
# fileread = 20190217_161815

# very good : score 0.000 - 0.002
# fileread = 20190217_161749

# path=~/falcons/data/internal/vision/multiCam/r1
# new keeperframe
# fileread = 20190223_135008


# path=~/falcons/data/internal/vision/multiCam/r7
# fileread = 20190402_185039

# path=~/falcons/data/internal/vision/multiCam/r2
# fileread = 20190404_224958

# ## porto 2019 ##
# path=~/falcons/data/internal/vision/multiCam/r6
# fileread = 20190426_191317
# fileread = 20190426_191806
# fileread = 20190426_191818
# fileread = 20190426_191759
# path=/home/robocup/falcons/code/tools/multiCamViewer/build

# techunited robot 3 + eric (false balls)
# path=~/falcons/data/internal/vision/multiCam/r1
# fileread = 20190427_160053

# path=~/falcons/code/tools/multiCamViewer/build
# fileread = 20190426_191232
# fileread = 20190426_191739
# fileread = 20190426_191232
# fileread = 20190426_191201
# fileread = 20190426_191205
# fileread = 20190426_191308

# de locht against tech united
# path=~/falcons/data/internal/vision/multiCam/r4
# good examples for wrong obstacle detections
# fileread = 20190606_212129
# fileread = 20190606_212209
# ball before robot
# fileread = 20190606_212312
# fileread = 20190606_212310
# fileread = 20190606_211932
# fileread = 20190606_211836
# fileread = 20190606_211839

# ## Sydney ##
path=~/falcons/data/internal/vision/multiCam/r2
# fileread = 20190702_062150
# fileread = 20190703_051051
fileread = 20190706_061811

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
	@$(JPG_TO_RGB) -i $(path)/cam0_$(fileread).jpg -o cam0Image.rgb
#	@cp ~/falcons/data/internal/vision/multiCam/r6/grab0_$(GRAB).rgb cam0Image.rgb
cam1Image.rgb: $(JPG_TO_RGB) Makefile
	@$(JPG_TO_RGB) -i $(path)/cam1_$(fileread).jpg -o cam1Image.rgb
#	@cp ~/falcons/data/internal/vision/multiCam/r6/grab1_$(GRAB).rgb cam1Image.rgb
cam2Image.rgb: $(JPG_TO_RGB) Makefile
	@$(JPG_TO_RGB) -i $(path)/cam2_$(fileread).jpg -o cam2Image.rgb
#	@cp ~/falcons/data/internal/vision/multiCam/r6/grab2_$(GRAB).rgb cam2Image.rgb
cam3Image.rgb: $(JPG_TO_RGB) Makefile
	@$(JPG_TO_RGB) -i $(path)/cam3_$(fileread).jpg -o cam3Image.rgb
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
