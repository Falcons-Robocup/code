 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * AbstractInterpolator.cpp
 *
 *  Created on: Dec 2019
 *      Author: Jan Feitsma
 */

#include "ext/AbstractInterpolator.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>


AbstractInterpolator::AbstractInterpolator()
{
}

AbstractInterpolator::~AbstractInterpolator()
{
}

void AbstractInterpolator::load(std::string const &filename)
{
    std::ifstream infile(filename.c_str());
    std::string line;
    while (std::getline(infile, line))
    {
        // ignore empty lines
        if (line.size() == 0) continue;
        // allow #comments lines
        if (line[0] == '#') continue; 
        std::istringstream iss(line);
        float a, b;
        if (!(iss >> a >> b)) break; // error
        // process pair (a,b)
        feed(a, b);
    }
}

void AbstractInterpolator::clear()
{
    _data.clear();
}

void AbstractInterpolator::feed(float x, float y)
{
    _data[x] = y;
}

void AbstractInterpolator::writeSampleData(std::string const &filename)
{
    std::ofstream outfile(filename.c_str());
    for (auto it = _data.begin(); it != _data.end(); ++it)
    {
        float x = it->first;
        float y = it->second;
        outfile << std::fixed << std::setw(8) << std::setprecision(3) << x << " ";
        outfile << std::fixed << std::setw(8) << std::setprecision(3) << y << std::endl;
    }
}

void AbstractInterpolator::writeInterpolatedCurve(float xmin, float xmax, float dx, std::string const &filename)
{
    std::ofstream outfile(filename.c_str());
    for (float x = xmin; x <= xmax; x += dx)
    {
        float y = evaluate(x);
        outfile << std::fixed << std::setw(8) << std::setprecision(3) << x << " ";
        outfile << std::fixed << std::setw(8) << std::setprecision(3) << y << std::endl;
    }
}

