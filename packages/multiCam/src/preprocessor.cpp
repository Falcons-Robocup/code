// Copyright 2018-2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
