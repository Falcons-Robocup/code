 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModelStimulator.cpp
 *
 *  Created on: Dec 2018
 *      Author: Jan Feitsma
 */


// system includes
#include <cstdio>

// other Falcons packages
#include "FalconsCommon.h"

// internal includes
#include "cWorldModelStimulator.hpp"

cWorldModelStimulator::cWorldModelStimulator(int agentId, std::string inputFile, std::string outputFile)
{
    _inputFile = inputFile;
    _outputFile = outputFile;
    _agentId = agentId; // TODO I think we can strip this -- script sets env, wm knows itself which agent it is
    _component = "worldModel";
}

cWorldModelStimulator::~cWorldModelStimulator()
{
}

bool cWorldModelStimulator::checkFrame(tLogFrame const &frame)
{
    return true; // TODO OK?
    /*
    // we don't introspect the data, we use the fact that the logger writes per agent a local and a shared frame on some frequency
    // so whenever we see such a frame, we have to recalculate
    if ((_agentId == frame.agent) && (frame.shared == true))
    {
        // note: here we could store things for use in tick, if we'd need to
        return true;
    }
    return false;
    */
}

void cWorldModelStimulator::tick(rtime const &t)
{
    _worldModel.update(t);
    if (_verbosity >= 3)
    {
        diagWorldModel wmDiag = _worldModel.getDiagnostics();
        // TODO tprintf
        //printf("  inplay=%d active=[%s] #vis=%2d #motor=%2d #balls=%2d #obst=%2d", 
        //    wmDiag.inplay, wmDiag.teamActivity.c_str(), wmDiag.numVisionCandidates, wmDiag.numMotorDisplacementSamples, wmDiag.numBallTrackers, wmDiag.numObstacleTrackers);
    }
}

