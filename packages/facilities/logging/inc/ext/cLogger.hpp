 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cLogger.hpp
 *
 * Sample RTDB contents on some frequency and store it in a file,
 * so it can be used to playback & visualize later.
 *
 *  Created on: Aug 11, 2018
 *      Author: Jan Feitsma
 */

#ifndef CLOGGER_HPP_
#define CLOGGER_HPP_


#include <string>
#include <vector>
#include <map>
#include <fstream>
#include "cLogFileWriter.hpp"
#include "FalconsRtDB2.hpp"


class cLogger
{
public:
    cLogger();
    ~cLogger();

    void setFrequency(int frequency);
    std::string createFileName();
    void writeToFile(std::string filename = "auto");
    bool makeFrame(tLogFrame &frame, rtime const &t);
    void monitor(); // loop around tick()
    bool tickNow(); // poked from monitor
    bool tick(rtime const &t); // handy for stimulation

private:
    RtDB2* _rtdb = NULL;
    int _frequency;
    cLogFileWriter *_logFile = NULL;
    rtime _t0; // start
    rtime _te; // end
    tLogHeader _header;

};

#endif

