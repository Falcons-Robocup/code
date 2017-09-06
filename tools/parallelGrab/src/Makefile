# Copyright Andre Pool
# Licensed under the Apache License version 2.0
# You may not use this file except in compliance with this License
# You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

TARGET = parallelGrab
OBJECTS = parallelGrab.o

MAKEFLAGS = -j8

CFLAGS =  -g3 -O0 -Wall -ansi -Wextra -m64 -Wpedantic -Wno-unused-but-set-variable

LDFLAGS = -L/usr/local/lib 
LDLIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lpthread

INC = -I../inc

all: $(TARGET) sf sv
	@./$(TARGET)

%.o: ../src/%.cpp Makefile
	g++ $(CFLAGS) $(INC) -o $@ -c $<

$(TARGET): $(OBJECTS)
	g++ $(LDFLAGS) -o $@ $^ $(LDLIBS)

sa:
	v4l2-ctl --list-formats-ext --device /dev/video0

sf:
	@v4l2-ctl --all --device /dev/video0 | grep Frames 2>/dev/null || true
	@v4l2-ctl --all --device /dev/video1 | grep Frames 2>/dev/null || true
	@v4l2-ctl --all --device /dev/video2 | grep Frames 2>/dev/null || true
	@v4l2-ctl --all --device /dev/video3 | grep Frames 2>/dev/null || true
	@v4l2-ctl --all --device /dev/video4 | grep Frames 2>/dev/null || true


sv:
	@ls -ltr /dev/video*

ss:
	@v4l2-ctl -V --device /dev/video0 2>/dev/null || true
	@v4l2-ctl -V --device /dev/video1 2>/dev/null || true
	@v4l2-ctl -V --device /dev/video2 2>/dev/null || true
	@v4l2-ctl -V --device /dev/video3 2>/dev/null || true
	@v4l2-ctl -V --device /dev/video4 2>/dev/null || true

s1920mjpg:
	v4l2-ctl --set-fmt-video=width=1920,height=1080,pixelformat=MJPG

s1920yuyv:
	v4l2-ctl --set-fmt-video=width=1920,height=1080,pixelformat=YUYV


# https://forums.opensuse.org/showthread.php/480974-How-to-capture-hardware-MJPEG-stream-using-mencoder

recf:
	ffmpeg -f video4linux2 -vcodec mjpeg -s 800x600 -r 30 -i /dev/video0 -vcodec copy -y mjpeg-ffmpeg.avi 2>&1 | grep -v Non-monotonous 

recv:
	vlc -I dummy v4l2:///dev/video0:width=800:height=600:fps=30 :v4l2-chroma=MJPG :v4l2-fps=30 --szout=file/asf:mjpeg-vlc.asf

recm:
	mencoder tv:// -tv driver=v4l2:width=800:height=600:fps=30:outfmt=mjpeg -nosound -ovc copy -o mjpeg-mencoder.avi

rect:
	@killall ffmpeg 2>/dev/null || true
	@rm -f mjpeg-ffmpeg*.avi
	ffmpeg -f video4linux2 -vcodec mjpeg -s 800x600 -r 30 -i /dev/video0 -vcodec copy -y mjpeg-ffmpeg0.avi 2>&1 | grep -v Non-monotonous &
	ffmpeg -f video4linux2 -vcodec mjpeg -s 800x600 -r 30 -i /dev/video1 -vcodec copy -y mjpeg-ffmpeg1.avi 2>&1 | grep -v Non-monotonous &
	ffmpeg -f video4linux2 -vcodec mjpeg -s 800x600 -r 30 -i /dev/video2 -vcodec copy -y mjpeg-ffmpeg2.avi 2>&1 | grep -v Non-monotonous &
	ffmpeg -f video4linux2 -vcodec mjpeg -s 800x600 -r 30 -i /dev/video3 -vcodec copy -y mjpeg-ffmpeg3.avi 2>&1 | grep -v Non-monotonous &
#	ffmpeg -f video4linux2 -vcodec mjpeg -s 800x600 -r 30 -i /dev/video4 -vcodec copy -y mjpeg-ffmpeg4.avi 2>&1 | grep -v Non-monotonous &
	sleep 5
	@killall ffmpeg 2>/dev/null || true
	@sync
	@du -h mjpeg-ffmpeg*.avi

diskfull:
	sudo rm -f /var/log/uvcdynctrl-udev.log

clean:
	@sync
	@rm -f *.o $(TARGET)


.PHONY: all sa sf ss s1920mjpg s1920yuyv recf recv recm rect diskfull clean

.SECONDARY:
