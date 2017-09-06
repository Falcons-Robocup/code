# Front Vision System for turtle5k robots

Andre Pool  
March 2016


## v4l2-ctl --list-devices
USB2.0 PC CAMERA (usb-0000:00:14.0-2):
        /dev/video0

HD Webcam C525 (usb-0000:00:14.0-3):
        /dev/video2

USB_Camera (usb-0000:00:14.0-4):
        /dev/video4

Vimicro USB2.0 UVC PC Camera (usb-0000:00:1a.0-1.6):
        /dev/video3


## v4l2-ctl -L --all (for for the Genius WideCam F100)
Driver Info (not using libv4l2):
        Driver name   : uvcvideo
        Card type     : USB_Camera
        Bus info      : usb-0000:00:14.0-3
        Driver version: 3.19.8
        Capabilities  : 0x84200001
                Video Capture
                Streaming
                Extended Pix Format
                Device Capabilities
        Device Caps   : 0x04200001
                Video Capture
                Streaming
                Extended Pix Format
Priority: 2
Video input : 0 (Camera 1: ok)
Format Video Capture:
        Width/Height  : 1280/720
        Pixel Format  : 'MJPG'
        Field         : None
        Bytes per Line: 0
        Size Image    : 1843200
        Colorspace    : SRGB
        Flags         : 
Crop Capability Video Capture:
        Bounds      : Left 0, Top 0, Width 1280, Height 720
        Default     : Left 0, Top 0, Width 1280, Height 720
        Pixel Aspect: 1/1
Streaming Parameters Video Capture:
        Capabilities     : timeperframe
        Frames per second: 30.000 (30/1)
        Read buffers     : 0

         backlight_compensation (int)    : min=0 max=1 step=1 default=0 value=0
                     brightness (int)    : min=-64 max=64 step=1 default=0 value=0
                       contrast (int)    : min=0 max=95 step=1 default=32 value=32
              exposure_absolute (int)    : min=50 max=10000 step=1 default=166 value=166 flags=inactive
                  exposure_auto (menu)   : min=0 max=3 default=3 value=3
                                1: Manual Mode
                                3: Aperture Priority Mode
                          gamma (int)    : min=100 max=300 step=1 default=165 value=165
                            hue (int)    : min=-2000 max=2000 step=1 default=0 value=0
           power_line_frequency (menu)   : min=0 max=2 default=1 value=1
                                0: Disabled
                                1: 50 Hz
                                2: 60 Hz
                     saturation (int)    : min=0 max=100 step=1 default=55 value=55
                      sharpness (int)    : min=1 max=7 step=1 default=2 value=2
      white_balance_temperature (int)    : min=2800 max=6500 step=10 default=4600 value=4600 flags=inactive
             auto_white_balance (bool)   : default=1 value=1


## v4l2-ctl --list-formats-ext (for for the Genius WideCam F100)
ioctl: VIDIOC_ENUM_FMT
        Index       : 0
        Type        : Video Capture
        Pixel Format: 'YUYV'
        Name        : YUV 4:2:2 (YUYV)
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 160x120
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 176x144
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 320x240
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 352x288
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 800x600
                        Interval: Discrete 0.100s (10.000 fps)
                Size: Discrete 1280x720
                        Interval: Discrete 0.125s (8.000 fps)
                Size: Discrete 1920x1080
                        Interval: Discrete 0.200s (5.000 fps)

        Index       : 1
        Type        : Video Capture
        Pixel Format: 'MJPG' (compressed)
        Name        : MJPEG
                Size: Discrete 640x480
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 160x120
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 176x144
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 320x240
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 352x288
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 800x600
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 1280x720
                        Interval: Discrete 0.033s (30.000 fps)
                Size: Discrete 1920x1080
                        Interval: Discrete 0.033s (30.000 fps)


