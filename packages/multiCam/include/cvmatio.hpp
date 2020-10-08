 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cvmatio.hpp
 *
 * Handy utilities to read/write binary cv::Mat objects from/to file.
 * Found on: https://github.com/takmin/BinaryCvMat
 *
 *     Created: May, 2018
 *      Author: Jan Feitsma
 */
 
 
#ifndef CVMATIO_HPP_
#define CVMATIO_HPP_


#include <fstream>
#include "opencv2/core/core.hpp"


// primary io functions
bool writeMatBinary(std::ofstream& ofs, const cv::Mat& out_mat);
bool readMatBinary(std::ifstream& ifs, cv::Mat& in_mat);


// converters for specialized types such as cv::Matx33d and cv::Vec4d
template <typename T>
bool writeMatBinary(std::ofstream& ofs, T out_mat)
{
    return writeMatBinary(ofs, cv::Mat(out_mat));
}

template <typename T>
bool readMatBinary(std::ifstream& ifs, T& in_mat)
{
    // read into Mat
    cv::Mat m;
    bool ok = readMatBinary(ifs, m);
    if (ok)
    {
        // convert to requested type, assign
        T tmp((double*)m.ptr()); // TODO typeof?
        in_mat = tmp;
    }
    return ok;
}

#endif /* CVMATIO_HPP_ */

