 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionMoveToPenaltyAngle.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#include "int/actions/cActionMoveToPenaltyAngle.hpp"

#include "int/stores/fieldDimensionsStore.hpp"
#include "int/cTeamplayCommon.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/utilities/trace.hpp"


using namespace teamplay;

cActionMoveToPenaltyAngle::cActionMoveToPenaltyAngle()
{
	boost::assign::insert( _actionParameters )
		("factor", std::make_pair(std::vector<std::string>{"float"}, true))
		;

	intention.actionType = actionEnum::MOVE_TO_PENALTY_ANGLE;
}


cActionMoveToPenaltyAngle::~cActionMoveToPenaltyAngle()
{

}

/* \brief Assume already at penalty spot. Turn to face left or right goal post randomly.
 *  Shoot at the goalpost assuming a given shooter accuracy factor
 * factor = 1 means we shoot exactly at the right angle (goalpost)
 * factor = 0 means we shoot at the goal center
 * if not given, assume 0.5 (in between goal center and goalpost
 *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong
 */
behTreeReturnEnum cActionMoveToPenaltyAngle::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {

    	Position2D myPos;
    	Position2D targetPos = {};
        cWorldModelInterface::getInstance().getOwnLocation(myPos);

        Point2D rightGoalPost = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALPOST_RIGHT);

        // parameter "factor": see above
        std::string factorStr("factor");
        float factor = 0.5;
        auto paramValPair = parameters.find(factorStr);
        if (paramValPair != parameters.end())
        {
            std::string factorVal = paramValPair->second;
            if (factorVal.compare(emptyValue) != 0)
            {
            	factor = std::stof(factorVal);
            }
        }

        // Compute angle from current facing angle to target
        // ToDo: add random left or right; for now assume right
		double angle = angle_between_two_points_0_2pi(myPos.x, myPos.y, rightGoalPost.x, rightGoalPost.y);
		targetPos.phi = factor * angle;

		intention.x = myPos.x;
		intention.y = myPos.y;
		sendIntention();

        if (positionReached(myPos.x, myPos.y, targetPos.phi))
		{
			// Target reached. Do nothing and return PASSED
			TRACE("cActionMoveToPenaltyAngle PASSED");
			moveTo(myPos.x, myPos.y, myPos.phi);
			return behTreeReturnEnum::PASSED;
		}
		else
		{
			// Target not reached. moveTo and return RUNNING
			moveTo(myPos.x, myPos.y, targetPos.phi);
			return behTreeReturnEnum::RUNNING;
		}

    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
