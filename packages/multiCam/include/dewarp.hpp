 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * dewarp.hpp
 *
 *  Created on: Jan 13, 2018
 *      Author: Jan Feitsma
 */


#ifndef DEWARP_HPP_
#define DEWARP_HPP_


#include <string>
#include <map>
#include <vector>
#include <tuple>
#include "opencv2/core/core.hpp"


class Dewarper
{

public:
    // robotId and camera have to be provided to figure out the calibration file(s)
    // if flag is set, automatically newest calibration files will be downloaded / used
    // based on provided timestamp, which is default taken to be 'newest'
    Dewarper(int robotId, int cam, bool autoUpdate = true, double timestamp = 1e99);
    ~Dewarper();

    // init time
    void readBIN(std::string const &filename); // read from binary file

    // run time

    // 'floor' transformation is designed for localization (white pixels)
    // it might also be used for obstacles (since we can safely assume these are not flying)
    // it should NOT be used for balls, instead see 'front' transformation
    // input: pixel location
    // output: success boolean and RCS coordinates in mm if successful
    bool transformFloor(uint16_t xPixel, uint16_t yPixel, int16_t &xField, int16_t &yField) const;

    // 'front' transformation is designed for balls
    // it basically takes out the intrinsic lens warp, so correct angles can be calculated
    // even when balls are airborne (which is often)
    // but it also corrects for small mounting tilts
    // this might also be used for obstacles, although floor transformation might be better?
    // input: pixel location
    // output: success boolean and spherical angles in radians
    bool transformFront(uint16_t xPixel, uint16_t yPixel, float &azimuth, float &elevation) const;

    // (test) utilities
    bool rangeCheck(uint16_t xPixel, uint16_t yPixel) const;
    std::vector<cv::Point> calcIsolineAzimuth(float az, int stride = 1);
    std::vector<cv::Point> calcIsolineElevation(float el, int stride = 1);

private:
    int _pixelRows, _pixelCols; // size of lookup tables, allowed range for input pixels
    cv::Mat _floorLookupX, _floorLookupY; // lookup table for Floor transformation: pixel to int RCS offset (x,y) in mm
    cv::Mat _frontLookupAz, _frontLookupEl; // lookup table for Front transformation: pixel to float angles (az,el)

    // with these calibrated fisheye matrices K and D, we could inline
    // correct the distortion in the diagnostics viewers using cv::fisheye::undistortImage
    // but for that, we need to upgrade to OpenCV3 - TODO
    cv::Matx33d _K;
    cv::Vec4d _D;

    // floor perspective homography, stored in calibration file but not used
    cv::Mat _Hf;

    // helpers
    void updateCalibrationFiles();
    std::string selectCalibrationFile(int robotId, int cam, double timestamp);
    std::map<std::string, std::vector<cv::Point>> _isolineCache; // isolines are expensive to calculate and never change

};

#endif /* DEWARP_HPP_ */

