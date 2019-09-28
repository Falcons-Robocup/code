 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * mLogdump.cpp
 *
 * Dump log to stdout.
 *
 *  Created on: January 2019
 *      Author: Jan Feitsma
 */


#include "ext/cLogFileReader.hpp"
#include <iostream>
#include <fstream>
#include <set>


int main(int argc, char **argv)
{
    if (argc == 1)
    {
        std::cerr << "ERROR: no logfile given" << std::endl;
        return 1;
    }
    
    // TODO option parsing: select agent, timeframe, etc
    
    // prepare
    cLogFileReader logFile(argv[1]);
    tLogHeader header = logFile.getHeader();
    int frameCounter = 0;
    tLogFrame frame;
    
    // heading
    std::cout << " frame      age  size                  timestamp" << std::endl;
    
    // run
    while (logFile.getFrame(frame))
    {
        frameCounter++;
        std::cout << " " << std::setw(5) << frameCounter;
        std::cout << " " << std::setw(8) << std::fixed << std::setprecision(2) << frame.age;
        std::cout << " " << std::setw(5) << (int)frame.data.size();
        rtime t = header.creation + frame.age;
        std::cout << " " << t.toStr();
        // TODO dump all keys? require RTDB API change
        std::cout << std::endl;
    }

    return 0;
}

