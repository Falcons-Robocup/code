# JFEI may 2018

TARGET = optiCal
UTIL = upgradeCalibration
OBJECTS = optiCal.o optiCalGUI.o optiCal.o CameraCalibrator.o multiCamVideoFeed.o camSysReceive.o cvmatio.o

MAKEFLAGS = -j8

CFLAGS = -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -DNOROS

# dependencies to Falcons common (tracing, Position2D + transformations) and openCV3

LDFLAGS = -pthread -L/usr/local/lib \
    -L/home/robocup/falcons/code/build/packages/facilities/common \
    -Wl,-rpath,/home/robocup/falcons/code/build/packages/facilities/common \
    -L/opt/ros/kinetic/lib/x86_64-linux-gnu
LDLIBS = -lpthread -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lopencv_calib3d -lopencv_videoio -lFalconsCommon

# too bad we cannot / may not use rosmake for this. It took some effort to put together this Makefile, but more importantly, it is not very maintainable...

INC = -I../include \
    -I../../../packages/multiCam/include \
    -I/home/robocup/falcons/code/packages/facilities/common/include/ext \
    -I/usr/include/opencv4

all: $(TARGET) $(UTIL)
	@nice -n 15 ./$(TARGET) /home/robocup/falcons/data/internal/vision/multiCam/chessBoard0_12.jpg

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

%.o: ../../../packages/multiCam/src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS) $(TARGET)Main.o
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
	@rm -f *.o $(TARGET)
	@rm -f *.jpg
	@sync

.PHONY: all clean eth

.SECONDARY:
