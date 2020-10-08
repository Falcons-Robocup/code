 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef DIAGPATHPLANNING_HPP_
#define DIAGPATHPLANNING_HPP_

#include "wayPoint.hpp"
#include "forbiddenArea.hpp"
#include "PIDTerms.hpp"

#include "RtDB2.h" // required for serialization


struct diagPIDstate
{
    PIDTerms x;
    PIDTerms y;
    PIDTerms Rz;
    SERIALIZE_DATA(x, y, Rz);
};


struct diagPathPlanning
{
    std::vector<wayPoint> path; // can contain a single target, or no target, or even an extra intermediate (sub-)target
    std::vector<forbiddenArea> forbiddenAreas;
    pose                  distanceToSubTargetRCS; // for kstplot_motion
    pose                  accelerationRCS; // for kstplot_motion
    std::vector<bool>     isAccelerating;
    std::vector<bool>     accelerationClipping;
    int                   numCalculatedObstacles;
    bool                  shortStroke;
    std::vector<bool>     deadzone;
    diagPIDstate          pid;

    SERIALIZE_DATA(path, forbiddenAreas, distanceToSubTargetRCS, accelerationRCS, isAccelerating, accelerationClipping, numCalculatedObstacles, shortStroke, deadzone, pid);
};

#endif

