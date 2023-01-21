// Copyright 2019-2022 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2018-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef RASPI_DEFAULTS_HPP
#define RASPI_DEFAULTS_HPP

#define CAMERA_RESET false
#define CAMERA_VERBOSE false

// camera clocks (imx219)
#define EXCK_FREQ_DEFAULT 0x1800   // 0x012a-0x012b : 24.00MHz = 24MHz + 0.00MHz = 0x18 + 0x00 = 0x1800 = 6144
#define VTPXCK_DIV_DEFAULT 5       // 0x0301
#define VTSYCK_DIV_DEFAULT 1       // 0x0303
#define PREPLLCK_VT_DIV_DEFAULT 3  // 0x0304
#define PREPLLCK_OP_DIV_DEFAULT 3  // 0x0305
#define PLL_VT_MPY_DEFAULT 0x0039  // 0x0306-0x0307 :  PLL video timing system multiplier value
#define OPPXCK_DIV_DEFAULT  0x0a   // 0x0309
#define OPSYCK_DIV_DEFAULT  0x01   // 0x030b
#define PLL_OP_MPY_DEFAULT  0x0072 // 0x030c-0x030d : PLL output system multiplier value

// camera control
#define TEST_PATTERN 0
#define ANALOG_GAIN 229
#define ANALOG_GAIN_JETSON 171
// TODO: try lower shutter value in the Locht (test with fast rotating robot)
#define SHUTTER 400 // 0x015a : 0x0523, 16 bits, course integration time, use this one to compensate for the 50Hz mains frequency
#define SHUTTER_JETSON 1057 // 0x015a
#define BLACK_LEVEL 50 // 0xd1ea : 0x0040, 10 bits, DT_PEDESTAL
#define BLACK_LEVEL_JETSON 64 // 0xd1ea: 10 bits, DT_PEDESTAL

// camera dimensions
// number of effective pixels 3296 x 2480
// number of active pixels 3280 x 2464
// #define ROI_WIDTH 3280/4 // 820
// raspidYuv Y pitch 800, Y height 608, UV pitch 400, UV Height 304
#define ROI_WIDTH (32*25) // 800

// #define ROI_HEIGHT 2464/4 // 616
#define ROI_HEIGHT (32*19) // 608
#define FLOOR_WIDTH 400

// raspi configured for x2 binning :
//   800 * 2 = 1600
//   608 * 2 = 1216

// value set by RaspiVidYUV for 800x608
// 0x0160 : 0x052a 0x0d78 0x0000 0x0ccf : 0x0000 0x099f 0x0668 0x04d0
// 0x0170 : 0x0101 0x0300 0x0101 0x0000 : 0x0000 0x0000 0x0000 0x0000

// the native resolution is 3280 x 2464
// X_START, X_END, Y_START and Y_END are without binning and for full frame they should be: 0, 3279, 0 and 2463, as set below
// X_SIZE and Y_SIZE are after the x2 binning and for full frame should be 3280/2 = 1640 and 2464/2 = 1232, as set below
// TODO: investigate why lines 1322 (all other things below make sense)
// would expect 3280 + 32 (0x20)(frame blanking) for the lines
// apparently it is used to control the fps
#define LINES 1322   // 0x0160-0x0161 : 0x052a = 1322, FRM_LENGTH_A : for 40 fps
// #define LINES_JETSON 1763   // 0x0160-0x0161 : FRM_LENGTH_A : for 30 fps
#define LINES_JETSON 7557   // 0x0160-0x0161 : FRM_LENGTH_A : for 7 fps
// #define LINES_JETSON (3280+32)   // 0x0160-0x0161 : FRM_LENGTH_A :
#define PIXELS 3448  // 0x0162-0x0163 : 0x0d78 = 3448, LINE_LENGTH_A : 3280 (active pixels) + 168 (0xa8)(line blanking)
#define X_START 0    // 0x0164-0x0165 : 0x0000 =    0, X_ADD_STA_A : X-address on the top left corner of the visible pixel data, default 0
#define X_END 3279   // 0x0166-0x0167 : 0x0ccf = 3279, X_ADD_END_A : X-address on the bottom right corner of the visible pixel data, default 3279
#define X_SIZE 1640  // 0x016c-0x016d : 0x0668 = 1640, X_OUTPUT_SIZE_A : width of output image 3280/2 = 1640
#define Y_START 0    // 0x0168-0x0169 : 0x0000 =    0, Y_ADD_STA_A : Y-address on the top left left corner of the visible pixel data, default 0
#define Y_END 2463   // 0x016a-0x016b : 0x099f = 2463, Y_ADD_END_A : Y-address on the bottom right corner of the visible pixel data : default 2463
#define Y_SIZE 1232  // 0x016e-0x016f : 0x04d0 = 1232, Y_OUTPUT_SIZE_A : height of output image 2464/2 = 1232
#define IMAGE_ORIENTATION 0 // 0x0172   : 0x00, bit[0] : horizontal, bit[1] : vertical

// line point detection
#define LINE_VAL_MIN 170
#define LINE_SAT_MAX 40 // low for ball in ball handler, high for lines far away
#define LINE_TRANSFER_PIXELS_MAX 32 // needs to large for close by white lines
#define LINE_FLOOR_WINDOW_SIZE 15
#define LINE_FLOOR_PIXELS_MIN 10
#define LINE_WINDOW_SIZE 31 // need to be larger then 15, because there might be non white pixels on the line
#define LINE_PIXELS_MIN 0 // small to be able to catch far away pixels

// ball point detection
#define BALL_VAL_MIN 150 // test with shadow area
#define BALL_SAT_MIN 65 // low value for bright ball (in ball handler)
#define BALL_HUE_MIN 15 // low (below 17) is orange, test for near by, test for TechUnited, yellow is 60 degrees => 60/2=30, 10 results into problems with orange TechUnited cover
#define BALL_HUE_MAX 35 // high is green, test for far away
#define BALL_WINDOW_SIZE 30
#define BALL_PIXELS_MIN 15 // TODO: set larger (check values Portugal 2019) and use BALL_FAR for smaller (to prevent orange to green)
#define BALL_FALSE_PIXELS_MAX 5

// ball far point detection
#define BALL_FAR_VAL_MIN 100 // test with shadow area
#define BALL_FAR_SAT_MIN 115
#define BALL_FAR_HUE_MIN 40 // low is orange, test for near by, test for TechUnited, yellow is 60 degrees => 60/2=30, 10 results into problems with orange TechUnited cover
#define BALL_FAR_HUE_MAX 55 // high is green, test for far away
#define BALL_FAR_WINDOW_SIZE 4
#define BALL_FAR_PIXELS_MIN 2
#define BALL_FAR_FALSE_PIXELS_MAX 2

// #define WHITE_BLACK_BALL_SEARCH
#ifdef WHITE_BLACK_BALL_SEARCH
#define BALL_WHITE_WINDOW_SIZE 10
#define BALL_WHITE_PIXELS_MIN 5
#define BALL_BLACK_WINDOW_SIZE 5
#define BALL_BLACK_PIXELS_MIN 2
#define BALL_WHITE_BLACK_FALSE_PIXELS_MAX 2
#endif

// floor point detection
#define FLOOR_VAL_MIN 50 // TODO: set higher, so far away line points will not be eaten up by floor (see Portugal 2019 values)
#define FLOOR_SAT_MIN 65 // verify if not eating to much white lines, so should be high, otherwise white lines will become green, or low when sun spots are recognized as lines
#define FLOOR_HUE_MIN 57 // floor is 200 degrees => 200/2=100
#define FLOOR_HUE_MAX 77 // above 95 is blue

// obstacle point detection
// TODO: needs to be calibrated
#define OBSTACLE_VAL_MAX 30 // typically lower then FLOOR VAL MIN
#define OBSTACLE_SAT_MAX 255 // dark obstacles might have a high saturation value
#define OBSTACLE_FLOOR_WINDOW_SIZE 20
#define OBSTACLE_FLOOR_PIXELS_MIN 10
#define OBSTACLE_LINE_WINDOW_SIZE 10
#define OBSTACLE_LINE_PIXELS_MIN 5
#define OBSTACLE_BALL_WINDOW_SIZE 10
#define OBSTACLE_BALL_PIXELS_MIN 5
#define OBSTACLE_TRANSFER_PIXELS_MAX 20
#define OBSTACLE_WINDOW_SIZE 30
#define OBSTACLE_PIXELS_MIN 5

// grabber values
// the red and blue gain are divided by 100 before sending to the grabber

// JFEI: change these, they represent the white balance (green is fixed)
// use cdmv alias to show the camera stream
// and use cdmc alias (choose grabber window) to try out different values
// it is "good enough" if the lines are white (no color visible)
// after determine a good red and blue define them over here
// and run alias cdrs (to change to setup raspi setup directory)
// and command ./copyBuildRaspi (to update the boards)
// optional run from other terminal ssh -t cam0 "tail -f log" to checkout what is going on

// NOTE: if there is an error on the boards about wrong packet, first run to ./copyBuildRaspi to update the binary on the board
// if it then still does not solve the issue also try a ./raspiReboot

#define GRABBER_RED_GAIN 200
#define GRABBER_BLUE_GAIN 180

#endif
