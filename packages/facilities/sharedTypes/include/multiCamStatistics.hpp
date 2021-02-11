// Copyright 2019-2020 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * multiCamStatistics.hpp
 *
 *  Created on: May 6, 2019
 *      Author: Andre Pool
 *
 * containing the statistics from the 4 raspi boards
 */

#ifndef MULTI_CAM_STATISTICS_HPP_
#define MULTI_CAM_STATISTICS_HPP_

#include "uniqueObjectID.hpp"
#include "cameraEnum.hpp"
#include "objectColorEnum.hpp"

#include "RtDB2.h" // required for serialization


struct raspiStatSingle {

    uint32_t           frameCounter = 0;

    uint16_t           cpuUptime = 0;
    uint16_t           grabberUptime = 0;
    uint16_t           systemUptime = 0;
    uint16_t           analyzeUptime = 0;

    uint8_t            rebootReceivedAge = 0;

    float              cpuLoad = 0.0;

    uint8_t            cpuTemp = 0;
    uint8_t            cmosTemp = 0;
    uint8_t            gpuTemp = 0;

    bool               underVoltage = false;
    bool               underVoltageOccurred = false;
    bool               cpuThrottle = false;
    bool               cpuThrottleOccurred = false;
    bool               cpuFrequencyCap = false;
    bool               cpuFrequencyCapOccurred = false;
    
    SERIALIZE_DATA(frameCounter, cpuUptime, grabberUptime, systemUptime, analyzeUptime, rebootReceivedAge, cpuLoad, cpuTemp, cmosTemp, gpuTemp, underVoltage, underVoltageOccurred, cpuThrottle, cpuThrottleOccurred, cpuFrequencyCap, cpuFrequencyCapOccurred);
};

struct multiCamStatistics {
    uniqueObjectID     identifier;
    std::vector<raspiStatSingle> raspi;
    
    uint8_t            ballAmount = 0;
    uint8_t            obstacleAmount = 0;
    
    uint16_t           ballPossessionPixels = 0;
    
    uint16_t           linePoints = 0;
    
    float              multiCamUptime = 0.0;
    uint64_t           multiCamTime = 0;
    
    float              cam0Fps = 0.0;
    float              localizationFps = 0.0;
    
    uint32_t           cam0Frames = 0;
    uint32_t           localizationFrames = 0;
    
    
    SERIALIZE_DATA(identifier, raspi, ballAmount, obstacleAmount, ballPossessionPixels, linePoints, multiCamUptime, multiCamTime, cam0Fps, localizationFps, cam0Frames, localizationFrames);
};

#endif
