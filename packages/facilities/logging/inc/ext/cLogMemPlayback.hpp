 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPlayback.hpp
 *
 * Load a log file and playback it contents, writing into RTDB.
 *
 *  Created on: Aug 12, 2018
 *      Author: Jan Feitsma
 */

#ifndef CPLAYBACK_HPP_
#define CPLAYBACK_HPP_


#include <string>
#include <utility>
#include <fstream>
#include "tLogHeader.hpp"
#include "tLogFrame.hpp"
#include "ext/cDbConnection.hpp"


class cPlayback : public cDbConnection
{
public:
    cPlayback(std::string const &filename);
    ~cPlayback();
    
    // run entire log on standard speed
    void run();
    bool step(); // perform a step, return success
    
    // detailed control, meant for visualizer
    void seek(rtime t); // seek to given timestamp
    void setDt(float dt); // modify step speed
    
private:
    std::string _filename;
    tLogHeader _header;
    tLogFrame _nextFrame;
    std::map<std::pair<int, bool>, tLogFrame> _frameQueue;
    std::ifstream _file;
    float _dt = 0.1;
    rtime _t;
    void clearFrameQueue();
    void queueFrame(tLogFrame const &frame);
    bool putFrameQueue();
    void putFrame(tLogFrame const &frame);
    void openFile();
    
};

#endif

