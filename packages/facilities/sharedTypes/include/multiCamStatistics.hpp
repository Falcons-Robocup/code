 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
