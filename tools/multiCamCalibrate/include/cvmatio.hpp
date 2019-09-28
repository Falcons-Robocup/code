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

