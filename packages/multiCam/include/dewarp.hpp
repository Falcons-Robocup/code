 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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
#include <vector>
#include <tuple>
#include "opencv2/core/core.hpp"


class deWarper
{
     
  public:
    deWarper();
    ~deWarper();
    
    // init time
    void loadCSV(std::string const &filename); // load config file with samples
    void calculate(); // after loading the samples, this function should be called to construct the lookup table

    // run time
    cv::Mat transform(cv::Mat const &input) const; // transform an image
    void transform(uint16_t xPixel, uint16_t yPixel, int16_t &xField, int16_t &yField) const; // transform one pixel

    // test utilities
    bool verify() const; // sanity check on pixel transform on measurement samples
    float cacheSize() const; // memory consumption in MB
    void plotHoles() const; // check for gaps in calibration area (interpolate might miss some pixels)
    void writeBIN(std::string const &filename); // write to binary file
    void readBIN(std::string const &filename); // read from binary file
    
  private:
    cv::Mat _reverseLookupX, _reverseLookupY; // cache for cv::remap to achieve fast transform
    cv::Mat _forwardLookupX, _forwardLookupY; // cache for direct pixel transform
    std::vector<std::tuple<int, int, float, float, bool>> _samples; // pixel to field + valid bool
    int _gridRows, _gridCols;
    int _pixelRows, _pixelCols;
};

#endif /* DEWARP_HPP_ */

