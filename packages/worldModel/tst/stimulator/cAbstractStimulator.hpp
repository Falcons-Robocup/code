 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cAbstractStimulator.hpp
 *
 *  Created on: Dec 2018
 *      Author: Jan Feitsma
 *
 * A stimulator is a test utility which feeds an input log file (RTDB stream) into a component, 
 * stimulating it, capturing the resulting RTDB data into an output log file.
 * Any existing outputs in the input log are overwritten.
 * (Visualizer playback can be used to browse any log; python tooling can be used to perform checks.)
 *
 * This is useful when testing the behavior of one component, both regression and progression.
 */

#ifndef CABSTRACTSTIMULATOR_HPP_
#define CABSTRACTSTIMULATOR_HPP_

#include <vector>
#include <string>
#include "cLogPlayback.hpp"
#include "tLogFrame.hpp"

class cAbstractStimulator: public cLogPlayback
{
public:
    cAbstractStimulator();
    virtual ~cAbstractStimulator();
    
    // client functions
    virtual bool checkFrame(tLogFrame const &frame) = 0;
    virtual void tick(rtime const &t) = 0;
    
    // process the log (or subset, if applicable)
    void run();
    
    // configuration
    void setVerbosity(int v);

private:
    // recalculate the frame, keeping timestamps consistent 
    bool mergeStimFrame(tLogFrame &frame, tLogFrame const &newFrame, rtime const &t);
    RtDB2Frame convertFrameLog2Rtdb(tLogFrame const &frame);
    tLogFrame convertFrameRtdb2Log(float age, std::vector<RtDB2FrameItem> const &items);

protected:
    std::string              _inputFile;
    std::string              _outputFile;
    std::string              _component = "<unknown>";
    int                      _agentId = -1;
    rtime                    _tStart;
    rtime                    _tEnd;
    int                      _verbosity = 1;
};

#endif

