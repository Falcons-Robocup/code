// Copyright 2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CONFIG_HPP
#define CONFIG_HPP

// the bayer converter (software) reduces the pixels by half for width and height

// 608 is about 115 degrees
#define CAM_WIDTH 608 // max 608 (1216)

// 968 is about 180 degrees
// 800 is likely enough to recognize balls near the robot
#define CAM_HEIGHT 800 // max 968 (1936)
// #define CAM_HEIGHT 832 // max 968 (1936)
// #define CAM_HEIGHT 864 // max 968 (1936)
// #define CAM_HEIGHT 968 // max 968 (1936)

#endif // CONFIG_HPP
