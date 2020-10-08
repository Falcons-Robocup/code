 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * testDewarp.cpp
 *
 * Command-line utility to inspect optical calibration lookup tables, as used in dewarp.
 * Two arguments are required: a camera image (.jpg or .png) and the corresponding calibration file (.bin).
 * Just click to inspect map values.
 * 
 *  Created on: May, 2018
 *      Author: Jan Feitsma
 */

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "dewarp.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>




bool hasEnding (std::string const &fullString, std::string const &ending) {
    if (fullString.length() >= ending.length()) {
        return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
    } else {
        return false;
    }
}

void onMouse(const int event, const int x, const int y, int, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        Dewarper *dewarp = (Dewarper *)userdata;
        bool ok;
        int16_t fx, fy;
        ok = dewarp->transformFloor(x, y, fx, fy);
        printf("pixel (%5d,%5d) --> %s result floor (coordinates in RCS mm): (%6d,%6d)\n", x, y, (ok?" OK":"NOK"), fx, fy);
        float az, el;
        ok = dewarp->transformFront(x, y, az, el);
        printf("pixel (%5d,%5d) --> %s result front (angles in radians):     (%6.3f,%6.3f)\n", x, y, (ok?" OK":"NOK"), az, el);
    }
}

int main(int argc, char **argv)
{

    // prepare
    cv::Mat image;
    std::string filename;
    Dewarper dewarp(0, 0, false); // legacy mode: do not automatically load, instead expect readBIN will be called
    std::string windowName = argv[0];
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(windowName, onMouse, &dewarp);
        
    // arg parsing
    for (int it = 1; it < argc; ++it)
    {
        std::string arg = argv[it];
        if (arg.size() > 3)
        {
            if (hasEnding(arg, ".bin"))
            {
                dewarp.readBIN(arg);
            }
            if (hasEnding(arg, ".jpg") || hasEnding(arg, ".png"))
            {
                image = cv::imread(arg, 1);
                cv::imshow(windowName, image);
            }
        }
    }
    
    // GUI loop
    for (;;)
    {
        int key = cv::waitKey(30);
        if (key >= 0)
        {
            if ((key == 27) || (key == 'q')) 
            {
                break; // escape / quit
            }
        }
    }

    return 0;
}

