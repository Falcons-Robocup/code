 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionAvoidPOI.cpp
 *
 *  Created on: Jun 27, 2016
 *      Author: Martijn van Veen
 */

#include "int/actions/cActionAvoidPOI.hpp"
#include "int/stores/ballStore.hpp"
#include "cDiagnostics.hpp"


cActionAvoidPOI::cActionAvoidPOI()
{
    boost::assign::insert( _actionParameters )
    ("target", std::make_pair(defaultPOI, false) )
        ;
    _intention.action = actionTypeEnum::UNKNOWN;
}

cActionAvoidPOI::~cActionAvoidPOI()
{

}

// Add POI as obstacle
// Returns: PASSED    if the obstacle is set in the ballstore (which will be sent to pathplanning if a move action occurs)
//          FAILED    if something went wrong // can this happen/does worldmodel tell us?
//			RUNNING   is never returned
behTreeReturnEnum cActionAvoidPOI::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        std::string targetStr("target");
    	boost::optional<Position2D> target = getPos2DFromStr(parameters, targetStr);

        if (target)
        {
            Position2D targetPos = *target;

            _intention.position.x = targetPos.x;
            _intention.position.y = targetPos.y;
            sendIntention();

            teamplay::ballStore::getBall().avoid();
            TRACE("cActionAvoidPOI PASSED");
            return behTreeReturnEnum::PASSED;
        }
        else
        {
            // No valid POI given to avoid
        	TRACE("cActionAvoidPOI Failed: no valid POI given to avoid");
            return behTreeReturnEnum::PASSED;
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionAvoidPOI::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
