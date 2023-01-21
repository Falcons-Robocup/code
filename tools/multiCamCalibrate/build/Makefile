# JFEI may 2018

TARGET = optiCalMain
UTIL = upgradeCalibration
OBJECTS = optiCal.o optiCalGUI.o optiCal.o CameraCalibrator.o multiCamVideoFeed.o camSysReceive.o cvmatio.o grabber.o

MAKEFLAGS = -j8

CFLAGS  = -O3 # # using -O0 will not generate relevant warnings
CFLAGS += -march=native -ansi -Wall -Wextra -Wpedantic -Werror
# CFLAGS += -std=c++17 # c++17 is same as c++1z
CFLAGS += -std=c++2a # c++2a will become c++20 in gcc version 10
CFLAGS += -pipe # speedup compilation process
CFLAGS += -DNOROS # exclude ros related code from multiCam sources

LDFLAGS  = -Wl,--enable-new-dtags # create new dynamic tags in ELF
LDFLAGS += -Wl,-rpath,/opt/pylon/lib # add the pylon library path to the runtime library search path
LDFLAGS += -Wl,-rpath,../../../build/packages/facilities/common # add the FalconsCommon library path to the runtime library search path
LDFLAGS += -Wl,--export-dynamic # add all symbols to the dynamic symbol table
LDFLAGS += -pthread

LDLIBS  = -L/opt/pylon/lib # tell the linker where it can find the pylon libraries
LDLIBS += -L../../../build/packages/facilities/common
# LDLIBS += -L/opt/ros/kinetic/lib/x86_64-linux-gnu
LDLIBS += -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio
LDLIBS += -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon
LDLIBS += -lpthread
LDLIBS += -lFalconsCommon

INC  = -I../include
INC += -I../../../vision/pylonToOpenCv
INC += -I../../../packages/multiCam/include
INC += -I../../../packages/facilities/common/include/ext
INC += -I/usr/include/opencv4
INC += -I/opt/pylon/include

all: pylon

# use for 4 x Raspberry pi camera's)
multiCam: $(TARGET) $(UTIL)
	@nice -n 15 ./$(TARGET) /home/robocup/falcons/data/internal/vision/multiCam/chessBoard0_12.jpg

# use this target for genius f100 calibration (used for the human detection test)
usb: $(TARGET) $(UTIL)
	@nice -n 15 ./$(TARGET) usb

# use for 4 x Basler Pylon camera's
pylon: $(TARGET)
	@nice -n 15 ./$(TARGET) pylon

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../../../packages/multiCam/src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../../../vision/pylonToOpenCv/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS) $(TARGET).o
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(UTIL): $(OBJECTS) $(UTIL).o
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)

ETHERNET=`multiCamNet`
eth:
	@echo "## use $(ETHERNET) for the multicast data ##"
	@ip link show $(ETHERNET) | grep -q MULTICAST || sudo ip link set dev $(ETHERNET) multicast on
	@route | grep "224.16.16.0" | grep -q " lo" && sudo route del -net 224.16.16.0 netmask 255.255.255.0 || true
	@route | grep "224.16.16.0" | grep -q " $(ETHERNET)" || sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev $(ETHERNET)

clean:
	@rm -f *.o $(TARGET) $(UTIL)
	@rm -f *.jpg
	@sync

.PHONY: all clean eth

.SECONDARY:


