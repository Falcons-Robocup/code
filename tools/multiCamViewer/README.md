# Viewer of the raw RGB data stream from the multiCam camera's

There are currently 4 camera's

The camera's transmit the stream to: 
 - multicast IP group 224.16.32.74
 - port 44444

The viewer will show them in 4 different windows:
 - cam0 : front
 - cam1 : left
 - cam2 : rear
 - cam3 : right

When all camera's are transmitting data, there will be some packet drop

This causes black horizontal sections in the viewer

This can also be seen by the warnings about dropped packets in the terminal

If a low packet drop is required (e.g. to capture complete moslty images), enable
or connect only 1 camera instead of 4

# Build
cd build
make
g++ -g3 -O0 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Wno-unused-but-set-variable -DNOROS -I../include -o cameraReceive.o -c ../src/cameraReceive.cpp
g++ -pthread -L/usr/local/lib -o multiCamViewer cameraReceive.o multiCamViewer.o -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread
INFO      : openCV version 2.4.1
INFO      : cameraReceive uses configuration file: /home/robocup/falcons/code/packages/multiCam/multiCam.yaml
INFO      : camera receive multicast group address 224.16.32.74 port 44444
INFO      : wait for data from the camera's
WARNING   : cam 3 received camera packet counter 173, but expected packet counter 170


To quit, press q or esc in gui window or ctrl-c in terminal

Andre Pool  
2018

