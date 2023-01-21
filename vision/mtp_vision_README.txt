howto for setting up the human with robot game
requires genius f100 webcam
laptop with decent GPU
darknet modified to crop genius f100 1280x720 to 960x720 (to better match with the
training data aspect ratio)
and then resize to 608x800 (to match with the dewarp, which might only work for 
the 608x800 resolution of the raspi)

Last updated June 25, by Jan and Andre

use the genius f100 usb webcam to capture humans (balls, objetcs, goal post, border)

1.  assure height of the camera is correct
2.  assure the camera is horizontal
3.  connect usb camera after Linux is booted (otherwise the webcam is on /dev/video0 and internal camera on /dev/video2)
4.  use cheese to verify the camera is working (and optional change the brightness)
5.  quit cheese
6.  cd ~/darknet; ./detect_genius_f100
	if image is black then probably the camera's are swapped
	if needed use ./detect_genius_f100 -c0 to select /dev/video0 instead of /dev/video2
7.  cd ~/falcons/code/vision/jsonReceiver; ./run
	the run script compiles the code, starts the application and selects robot 10
	robot 10 is used for the dewarp files related to the genius f100
	NOTE: be sure robot 10 is shown in this line: "INFO    using configuration for robot 10"
8.  fc ; export TURTLE5K_ROBOTNUMBER=7 ; processStart A7 visionMLA
9.  logcd; tail -f stdout_A_visionMLA_*.txt 
10. export TURTLE5K_ROBOTNUMBER=7 ; frun mixedTeamProtocol/adapter MTPExternalHumanMain -i 7 -r ATTACKER_ASSIST

NOTE: static setup hardcoded at the moment at (-6, -3) facing field center


Below log output examples


## jsonReceveiver log
robocup@coach [mtp_vision] jsonReceiver $  cd ~/falcons/code/vision/jsonReceiver; ./run
make: Nothing to be done for 'first'.
INFO    using configuration for robot 10
WARNING confguration image size 720x960 does match default camera size 600x800
INFO    first wireless ethernet interface: wlp4s0
INFO    route: 224.16.32.0     0.0.0.0         255.255.255.0   U     0      0        0 wlp4s0
INFO    transmitting diagnostics on multicast group 224.16.32.74 port 45454
INFO    first wired ethernet interface: enp0s31f6
INFO    route: 224.16.16.0     0.0.0.0         255.255.255.0   U     0      0        0 enp0s31f6
INFO    transmitting export data on multicast group 224.16.16.16 port 46464
INFO    initializing dewarp ...
INFO    successfully read file /home/robocup/falcons/data/internal/vision/multiCam/calibration/20210625_155231_r10_cam0.bin
INFO    successfully read file /home/robocup/falcons/data/internal/vision/multiCam/calibration/20210625_155231_r10_cam1.bin
INFO    successfully read file /home/robocup/falcons/data/internal/vision/multiCam/calibration/20210625_155231_r10_cam2.bin
INFO    successfully read file /home/robocup/falcons/data/internal/vision/multiCam/calibration/20210625_155231_r10_cam3.bin
INFO    dewarp initialized
INFO    wait for connection with yolo
INFO    connected to yolo


## darknet log
robocup@coach [master] darknet $ ./detect_genius_f100 
chmod +x *.sh
./:
-c 2 -json_port 8070 -thresh 0.20 -ext_output
 CUDA-version: 11030 (11030), cuDNN: 8.2.1, GPU count: 1  
 OpenCV version: 4.2.0
Demo
 0 : compute_capability = 500, cudnn_half = 0, GPU: NVIDIA Quadro M2000M 
net.optimized_memory = 0 
mini_batch = 1, batch = 1, time_steps = 1, train = 0 
   layer   filters  size/strd(dil)      input                output
   0 Create CUDA-stream - 0 
 Create cudnn-handle 0 
conv     32       3 x 3/ 1    416 x 416 x   3 ->  416 x 416 x  32 0.299 BF
   1 conv     64       3 x 3/ 2    416 x 416 x  32 ->  208 x 208 x  64 1.595 BF
   2 conv     64       1 x 1/ 1    208 x 208 x  64 ->  208 x 208 x  64 0.354 BF
...
 159 conv   1024       3 x 3/ 1     13 x  13 x 512 ->   13 x  13 x1024 1.595 BF
 160 conv     30       1 x 1/ 1     13 x  13 x1024 ->   13 x  13 x  30 0.010 BF
 161 yolo
[yolo] params: iou loss: ciou (4), iou_norm: 0.07, obj_norm: 1.00, cls_norm: 1.00, delta_norm: 1.00, scale_x_y: 1.05
nms_kind: greedynms (1), beta = 0.600000 
Total BFLOPS 59.592 
avg_outputs = 490304 
 Allocate additional workspace_size = 3.12 MB 
Loading weights from yolov4_final.weights_20201112...
 seen 64, trained: 3840 K-images (60 Kilo-batches_64) 
Done! Loaded 162 layers from weights-file 
Webcam index: 2
[ WARN:0] global ../modules/videoio/src/cap_gstreamer.cpp (935) open OpenCV | GStreamer warning: Cannot query video position: status=0, value=-1, duration=-1
Video stream: 1280 x 720 
Objects:

 JSON-stream sent. 

FPS:0.0          AVG_FPS:0.0
Objects:

 JSON-stream sent. 
ball: 100%      (left_x:  675   top_y:  496   width:   52   height:   49)

FPS:0.4          AVG_FPS:0.0
Objects:

 JSON-stream sent. 
ball: 100%      (left_x:  675   top_y:  496   width:   52   height:   49)

...

## mlAdapter log file 
==> stdout_A_visionMLA_9.txt <==
networkName: 
databaseName: default
databasePath: /tmp/rtdb_teamA/0/default
2021-06-25,20:58:33.577448 INFO    listen on multicast group 224.16.16.16 port 46464
2021-06-25,20:58:34.263327 INFO    2021-06-25,20:58:33.263000 robot 10 cam 0 frame   1260 objects  2
2021-06-25,20:58:34.263369            human      conf 0.95 x 0.50 y 0.55 width 0.37 height 0.87 azimuth -0.01 elevation -0.12 radius 1.03
2021-06-25,20:58:34.263394            ball       conf 1.00 x 0.76 y 0.96 width 0.15 height 0.09 azimuth  0.50 elevation -0.61 radius 1.06
2021-06-25,20:58:39.076493 INFO    2021-06-25,20:58:38.076000 robot 10 cam 0 frame   1280 objects  2
2021-06-25,20:58:39.076524            human      conf 0.96 x 0.49 y 0.55 width 0.33 height 0.82 azimuth -0.04 elevation -0.12 radius 1.07
2021-06-25,20:58:39.076534            ball       conf 1.00 x 0.76 y 0.96 width 0.15 height 0.09 azimuth  0.50 elevation -0.61 radius 1.06
2021-06-25,20:58:43.887972 INFO    2021-06-25,20:58:42.887000 robot 10 cam 0 frame   1300 objects  3
2021-06-25,20:58:43.888009            human      conf 0.98 x 0.50 y 0.55 width 0.36 height 0.86 azimuth -0.01 elevation -0.12 radius 1.03
