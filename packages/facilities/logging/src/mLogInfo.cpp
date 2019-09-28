 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * mLogInfo.cpp
 *
 * Write log info to stdout.
 *
 *  Created on: January 2019
 *      Author: Jan Feitsma
 */


#include "ext/cLogFileReader.hpp"
#include <iostream>
#include <fstream>
#include <set>


std::ifstream::pos_type filesize(const char* filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    return in.tellg();
}


int main(int argc, char **argv)
{
    // need argument
    if (argc == 1)
    {
        std::cerr << "ERROR: no logfile given" << std::endl;
        return 1;
    }

    // help request?
    if (std::string(argv[1]) == std::string("-h"))
    {
        std::cout << "usage: rdlinfo [-h] rdlfile" << std::endl;
        std::cout << std::endl;
        std::cout << "Show info and statistics on given RDL logging file." << std::endl;
        return 0;
    }

    // prepare
    cLogFileReader logFile(argv[1]);
    tLogHeader header = logFile.getHeader();
    int numFrames = 0;
    tLogFrame frame;

    // TODO: automatically try to fix and reload, if duration is zero? (just call rdlfix)

    // run
    int totalDataSize = 0;
    int minDataSize = 999999;
    int maxDataSize = 0;
    while (logFile.getFrame(frame))
    {
        numFrames++;
        // inspect frame size
        int b = frame.data.size();// + 4; // 4 = string size descriptor
        if (b < minDataSize)
        {
            minDataSize = b;
        }
        if (b > maxDataSize)
        {
            maxDataSize = b;
        }
        totalDataSize += b;
    }

    // display basic statistics
    std::cout << "     filename: " << header.filename << std::endl; // does not have to be consistent with actual filename
    int sz = filesize(argv[1]);
    float mb = sz / 1024.0 / 1024.0;
    std::cout << "     filesize: " << sz << " [B] (" << std::fixed << std::setprecision(2) << mb << "MB)"<< std::endl;
    std::cout << "     hostname: " << header.hostname << std::endl;
    std::cout << "     creation: " << header.creation.toStr() << std::endl;
    std::cout << "     duration: " << std::fixed << std::setprecision(2) << header.duration << " [s]" << std::endl;
    std::cout << "    frequency: " << header.frequency << " [Hz]" << std::endl;
    std::cout << "   compressed: " << (header.compression ? "yes" : "no") << std::endl;
    std::cout << "    numFrames: " << numFrames << std::endl;

    // display advanced statistics
    std::cout << "  minDataSize: " << minDataSize << " [B]" << std::endl;
    std::cout << "  maxDataSize: " << maxDataSize << " [B]" << std::endl;
    std::cout << "  avgDataSize: " << std::fixed << std::setprecision(2) << (totalDataSize / 1.0 / numFrames) << " [B/frame]" << std::endl;
    std::cout << " avgFrameSize: " << std::fixed << std::setprecision(2) << (sz / 1.0 / numFrames) << " [B/frame]" << std::endl;
    std::cout << "  avgOverhead: " << std::fixed << std::setprecision(2) << ((sz - totalDataSize) / 1.0 / numFrames) << " [B/frame]" << std::endl;
    if (header.duration > 0)
    {
        std::cout << "  avgDataRate: " << std::fixed << std::setprecision(2) << (totalDataSize / 1024.0 / header.duration) << " [KB/s]" << std::endl;
    }
    return 0;
}

