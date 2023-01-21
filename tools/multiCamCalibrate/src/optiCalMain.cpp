/*
 * optiCalMain.cpp
 *
 * Optical Calibration.
 * Command-line utility to calibrate a multiCam assembly (camera 0,1,2,3 in sequence). 
 * Result is a set of binary calibration files in the data repo, which should then be configured to load from multiCam at init time.
 * 
 *  Created on: May 2018
 *      Author: Jan Feitsma
 */


#include "optiCal.hpp"
#include <string>



bool hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

int main(int argc, char **argv)
{

    // setup video source
    multiCamVideoFeed m;
    opticalCalibrator optiCal;
    
    // arg parsing
    for (int it = 1; it < argc; ++it)
    {
        std::string arg = argv[it];
        if (arg.size() > 2)
        {
            if (hasEnding(arg, ".bin"))
            {
                // select camera for landmarks
                int camId = (arg[arg.size()-5] - '0');
                optiCal.selectCamera(camId);
                optiCal.load(arg);
            }
            if (hasEnding(arg, ".jpg") || hasEnding(arg, ".png"))
            {
                m.setStill(arg);
            }
            if (hasEnding(arg, "usb") || hasEnding(arg, "USB"))
            {
                m.setUsb(2); // webcam on HP Zbook G3 available through /dev/video2
            }
            if (hasEnding(arg, "usb0") || hasEnding(arg, "USB0"))
            {
                m.setUsb(0);
            }
            if (hasEnding(arg, "usb1") || hasEnding(arg, "USB1"))
            {
                m.setUsb(1);
            }
            if (hasEnding(arg, "usb2") || hasEnding(arg, "USB2"))
            {
                m.setUsb(2);
            }
            if (hasEnding(arg, "pylon") )
            {
                optiCal.setPylon();
                m.setPylon();
            }
        }
    }
    
    
    // connect and run
    optiCal.connectVideo(&m);
    optiCal.run();
    
    return 0;
}

