 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// the preprocessor is now performed in the camera boards
// this class is currently only to calculate uptime and fps

#include "preprocessor.hpp"

using namespace std;
using namespace cv;

preprocessor::preprocessor(configurator *conf) {
    fps = 0;
    this->conf = conf;

    start = getTickCount();
    int timeSize = sizeof(time) / sizeof(time[0]);
    for( int ii=0; ii<timeSize; ii++ ) {
        time[ii] = 0; // fps will be presented as 0 until array is filled with data
    }
    count = 0;
}

void preprocessor::update(){
    // keep track of the last number of times when the localization was performed to be able to calculate the average localization FPS
    int timeSize = sizeof(time) / sizeof(time[0]);
    for( int ii=timeSize-1; ii>0 ; ii-- ) {
        time[ii] = time[ii-1];
    }
    time[0] = getTickCount();

    // calculate FPS
    uptime = (time[0] - start )/getTickFrequency();
    fps = (timeSize - 1) * 1.0 /((time[0] - time[timeSize-1])/getTickFrequency());

    // keep track how many frames have been processed
    count++;
}
