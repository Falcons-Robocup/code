 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * dewarp.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: Jan Feitsma
 */

#include "dewarp.hpp"
#include "cvmatio.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <sstream>
#include <fstream>



deWarper::deWarper()
{
}

deWarper::~deWarper()
{
}

void deWarper::loadCSV(std::string const &filename)
{
    // load config file with samples
    // each line contains a tuple (pixelX, pixelY, fieldX, fieldY)
    _samples.clear();
    // parse file, store grid calibration samples
    std::ifstream infile(filename.c_str());
    if (!infile)
    {
        throw std::runtime_error("file not found: " + filename);
    }
    std::string line;
    while (std::getline(infile, line))
    {
        int a = 0, b = 0; // pixel (x,y)
        float c = 0.0, d = 0.0; // field (x,y)
        std::string dummy;
        if (line.size() == 0) continue;
        if (line[0] == '#') continue;
        std::istringstream iss(line);
        if (line[0] == 'p')
        {
            // read pixel dimensions
            iss >> dummy >> _pixelCols >> _pixelRows;
            continue;
        }
        if (line[0] == 'g')
        {
            // read grid dimensions
            iss >> dummy >> _gridCols >> _gridRows;
            continue;
        }
        if (line[0] == 'i')
        {
            // invalid mark
            iss >> dummy >> c >> d;
            _samples.push_back(std::tuple<int, int, float, float, bool>(0, 0, c, d, false));
            continue;
        }
        if (!(iss >> a >> b >> c >> d)) continue;
        // process line data
        _samples.push_back(std::tuple<int, int, float, float, bool>(a, b, c, d, true));
    }

    // size check
    if ((int)_samples.size() != _gridCols * _gridRows)
    {
        fprintf(stderr, "grid size inconsistency (got %d samples, expected %d)\n", (int)_samples.size(), _gridCols * _gridRows);
        throw std::runtime_error("grid size inconsistency");
    }
    // don't forget to call calculate() afterwards
}

void deWarper::calculate()
{
    // sanity checks
    if (_samples.size() < 4)
    {
        throw std::runtime_error("too few samples");
    }
    // initialize maps
    cv::Mat errorMap(_pixelRows, _pixelCols, CV_32F, cv::Scalar(1e9));
    cv::Mat mapX(_pixelRows, _pixelCols, CV_32F, cv::Scalar(0));
    cv::Mat mapY(_pixelRows, _pixelCols, CV_32F, cv::Scalar(0));
    _forwardLookupX = cv::Mat(_pixelRows, _pixelCols, CV_16SC1, cv::Scalar(0)); // 0 is the default value for pixels which are either not interpolated or out of bounds
    _forwardLookupY = cv::Mat(_pixelRows, _pixelCols, CV_16SC1, cv::Scalar(0));
    // calculate grid 'width', to cope with negative values in 4th column, making a proper image
    int gridWidth = 0;
    for (auto it = _samples.begin(); it != _samples.end(); ++it)
    {
        gridWidth = std::max(gridWidth, abs(std::get<3>(*it)));
    }
    // grid scale for calibration/debug image - TODO dynamically
    float gridScale = 0.07;
    // calculate lookup table
    // make use of the lexical ordering of measurement samples
    bool rowMajor = false;
    int stepx = (rowMajor ? 1 : _gridRows);
    int stepy = (rowMajor ? _gridCols : 1);
    // TODO: a more advanced & complex method could be to use triangulation, then the calibration points would not have to specified in grid order
    // TODO: it feels weird that we 'have to' write out bilinear interpolation, should't we be able to use some standard opencv functions? cv::remap already does bilinear interpolation ... are we using it wrong? already googled for an hour ...
    // legend: index postfix g = grid, p = pixel, s = sample, f = field, prefix neighbours (north/east/south/west)
    for (int irowg = 1; irowg < _gridRows; ++irowg)
    {
        for (int icolg = 1; icolg < _gridCols; ++icolg)
        {
            // four neighbour indices
            int inws = (irowg-1) * stepy + (icolg-1) * stepx;
            int ines = (irowg-1) * stepy + icolg * stepx;
            int isws = irowg * stepy + (icolg-1) * stepx;
            int ises = irowg * stepy + icolg * stepx;
            // check if this cell is valid
            bool valid = std::get<4>(_samples[inws]) && std::get<4>(_samples[ines]) && std::get<4>(_samples[isws]) && std::get<4>(_samples[ises]);
            if (!valid) continue;
            // process the target grid in this cell
            int irows1 = std::get<3>(_samples[inws]);
            int irows2 = std::get<3>(_samples[ises]);
            int icols1 = std::get<2>(_samples[inws]);
            int icols2 = std::get<2>(_samples[ises]);
            float szx = icols2 - icols1;
            float szy = irows2 - irows1;
            for (int irowf = irows1; irowf <= irows2; ++irowf)
            {
                for (int icolf = icols1; icolf <= icols2; ++icolf)
                {
                    float wx = (icolf - icols1) / szx;
                    float wy = (irowf - irows1) / szy;
                    // bilinear interpolation to find source pixel
                    float pixelX = std::get<0>(_samples[inws]) * (1-wx) * (1-wy)
                                 + std::get<0>(_samples[ines]) * wx * (1-wy)
                                 + std::get<0>(_samples[isws]) * (1-wx) * wy
                                 + std::get<0>(_samples[ises]) * wx * wy;
                    float pixelY = std::get<1>(_samples[inws]) * (1-wx) * (1-wy)
                                 + std::get<1>(_samples[ines]) * wx * (1-wy)
                                 + std::get<1>(_samples[isws]) * (1-wx) * wy
                                 + std::get<1>(_samples[ises]) * wx * wy;
                    int irowp = (int)round(pixelY);
                    int icolp = (int)round(pixelX);
                    // sanity check
                    if (icolp < 0 || icolp >= _pixelCols || irowp < 0 || irowp >= _pixelRows)
                    {
                        fprintf(stderr, "pixel out of bounds: icolp=%d irowp=%d\n", icolp, irowp);
                        throw std::runtime_error("pixel out of bounds");
                    }
                    // check pixel quality, to make sure we select the best one
                    float pixelDistance2 = (pixelX - icolp) * (pixelX - icolp) + (pixelY - irowp) * (pixelY - irowp);
                    if (pixelDistance2 < errorMap.at<float>(irowp, icolp))
                    {
                        errorMap.at<float>(irowp, icolp) = pixelDistance2;
                        // store pixel for direct pixel transforms
                        _forwardLookupX.at<int16_t>(irowp, icolp) = icolf;
                        _forwardLookupY.at<int16_t>(irowp, icolp) = irowf;
                        // store the pixel for image transformation (calibration debugging only, not used during production)
                        if (1)
                        {
                            // apply offset and scaling to make possibly negative and large (mm resolution) visible in the image
                            int x = ((float)icolf +    0) * gridScale;
                            int y = ((float)irowf + gridWidth) * gridScale; // irowf can be negative, but pixels cannot
                            //fprintf(stderr, "irowg=%d icolg=%d irowp=%d icolp=%d irowf=%d icolf=%d x=%d y=%d pixelX=%.3f pixelY=%.3f\n", 
                            //      irowg, icolg, irowp, icolp, irowf, icolf, x, y, pixelX, pixelY);
                            if (x < 0 || x >= _pixelCols || y < 0 || y >= _pixelRows)
                            {
                                fprintf(stderr, "pixel out of bounds: x=%d y=%d\n", x, y);
                                throw std::runtime_error("pixel out of bounds");
                            }
                            mapX.at<float>(y, x) = pixelX;
                            mapY.at<float>(y, x) = pixelY;
                        }
                    }
                }
            }
        }
    }
    // TODO fill holes in forward lookup? given we interpolate based on 1 mm field resolution, we should be fine? see plotHoles()
    // convert maps for fast image transformation
    cv::convertMaps(mapX, mapY, _reverseLookupX, _reverseLookupY, CV_16SC2);
    // done
}

cv::Mat deWarper::transform(cv::Mat const &input) const
{
    // size checking
    if (input.rows != _pixelRows || input.cols != _pixelCols) 
    {
        fprintf(stderr, "image size inconsistency (expected (%d,%d), got (%d,%d))\n", _pixelRows, _pixelCols, input.rows, input.cols);
        throw std::runtime_error("image size inconsistency");
    }
    // transform given image
    cv::Mat result;
    cv::remap(input, result, _reverseLookupX, _reverseLookupY, CV_INTER_LINEAR);
    return result;
}

void deWarper::transform(uint16_t xPixel, uint16_t yPixel, int16_t &xField, int16_t &yField) const
{
    // range checking
    if (xPixel >= _pixelCols) 
    {
        fprintf(stderr, "xPixel (%d) out of bounds [%d,%d)", xPixel, 0, _pixelCols);
        throw std::runtime_error("xPixel out of bounds");
    }
    if (yPixel >= _pixelRows) 
    {
        fprintf(stderr, "yPixel (%d) out of bounds [%d,%d)", yPixel, 0, _pixelRows);
        throw std::runtime_error("yPixel out of bounds");
    }
    // calibration was done with a border, which we need to account for ... (TODO: get rid of this)
    int BORDER_SIZE = 120; 
    // transform the pixel
    xField = _forwardLookupX.at<int16_t>(yPixel+BORDER_SIZE, xPixel);
    yField = _forwardLookupY.at<int16_t>(yPixel+BORDER_SIZE, xPixel);
    // note: 0 is currently returned as default value for pixels which are either not interpolated or out of bounds
}

float deWarper::cacheSize() const
{
    return (_forwardLookupX.total() * _forwardLookupX.elemSize() + 
            _forwardLookupY.total() * _forwardLookupY.elemSize() + 
            _reverseLookupX.total() * _reverseLookupX.elemSize() + 
            _reverseLookupY.total() * _reverseLookupY.elemSize()) / 1024.0 / 1024.0;
}

void deWarper::plotHoles() const
{
    // TODO suggestion to make a black&white image of the holes
    // so we can visually check if all are outside the calibration area
    // for now: dump an ascii file
    std::string filename = "/var/tmp/dewarpCoverage.txt";
    FILE *fp = fopen(filename.c_str(), "w");
    for (int irowp = 0; irowp < _pixelRows; ++irowp)
    {
        for (int icolp = 0; icolp < _pixelCols; ++icolp)
        {
            if (_forwardLookupX.at<int16_t>(irowp, icolp) == 0 && _forwardLookupY.at<int16_t>(irowp, icolp) == 0)
            {
                fprintf(fp, "x"); // missing
            }
            else
            {
                fprintf(fp, "."); // ok
            }
        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    printf("file written: %s\n", filename.c_str());
}

bool deWarper::verify() const
{
    plotHoles();
    printf(" %11s %13s %13s %11s\n", "pixel", "field", "sample", "error");
    int16_t xPixel, yPixel;
    int16_t xField, yField;
    int16_t xSample, ySample;
    int xError, yError;
    int maxError = 0;
    bool valid;
    for (auto it = _samples.begin(); it != _samples.end(); ++it)
    {
        valid = std::get<4>(*it);
        if (valid)
        {
            xPixel = std::get<0>(*it);
            yPixel = std::get<1>(*it);
            xSample = std::get<2>(*it);
            ySample = std::get<3>(*it);
            transform(xPixel, yPixel, xField, yField);
            xError = abs(xField - xSample);
            yError = abs(yField - ySample);
            maxError = std::max(maxError, std::max(xError, yError));
            printf(" (%4d,%4d) (%5d,%5d) (%5d,%5d) (%4d,%4d)\n", xPixel, yPixel, xField, yField, xSample, ySample, xError, yError);
        }
    }
    return maxError == 0;
}

void deWarper::writeBIN(std::string const &filename)
{
    std::ofstream ofs(filename, std::ios::binary);
    // header: image dimensions
    ofs.write((char*)&_pixelCols, sizeof(_pixelCols));
    ofs.write((char*)&_pixelRows, sizeof(_pixelRows));
    // x and y lookups
    bool success = writeMatBinary(ofs, _forwardLookupX);
    if (!success)
    {
        throw std::runtime_error("failed to write _forwardLookupX as binary to file " + filename);
    }
    success = writeMatBinary(ofs, _forwardLookupY);
    if (!success)
    {
        throw std::runtime_error("failed to write _forwardLookupY as binary to file " + filename);
    }
}

void deWarper::readBIN(std::string const &filename)
{
    std::ifstream ifs(filename, std::ios::binary);
    // header: image dimensions
    ifs.read((char*)&_pixelCols, sizeof(_pixelCols));
    ifs.read((char*)&_pixelRows, sizeof(_pixelRows));
    printf("_pixelCols=%d _pixelRows=%d\n", _pixelCols, _pixelRows);
    // x and y lookups
    bool success = readMatBinary(ifs, _forwardLookupX);
    if (!success)
    {
        throw std::runtime_error("failed to read _forwardLookupX as binary from file " + filename);
    }
    success = readMatBinary(ifs, _forwardLookupY);
    if (!success)
    {
        throw std::runtime_error("failed to read _forwardLookupY as binary from file " + filename);
    }
    printf("successfully read file %s\n", filename.c_str());
}

