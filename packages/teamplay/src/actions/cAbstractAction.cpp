 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cAbstractAction.cpp
 *
 *  Created on: Apr 30, 2016
 *      Author: Erik Kouters
 */

#include "int/actions/cAbstractAction.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <stdexcept>
#include <FalconsCommon.h>
#include <timeConvert.hpp>

#include "int/cWorldModelInterface.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "int/rules/ruleAvoidAreas.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/types/intentionSync.hpp"
#include "int/stores/teamMatesStore.hpp"

/*
 * Include as last file
 * Undefs TRACE and creates its own
 */
#include "int/utilities/trace.hpp"

using namespace teamplay;

/* Static definitions */
static boost::shared_ptr<ruleAvoidAreas> rule_p;
static boost::shared_ptr<timer> timer_p;


cAbstractAction::cAbstractAction()
{
    hal = 0;
    pp = 0;
    sp = 0;
    intention.actionType = actionEnum::INVALID;
    intention.robotID = getRobotNumber();
    intention.x = 0.0;
    intention.y = 0.0;
    intention.timestamp = getTimeNow();

    timer_p.reset(new timer());
    rule_p.reset(new ruleAvoidAreas(timer_p));
}

cAbstractAction::~cAbstractAction()
{
    rule_p.reset();
    timer_p.reset();
}

void cAbstractAction::setHALInterface(cHALInterface* halInterface)
{
    hal = halInterface;
}

void cAbstractAction::setPathPlanningInterface (cPathPlanningInterface* pathPlanningInterface)
{
    pp = pathPlanningInterface;
}

void cAbstractAction::setShootPlanningInterface (cShootPlanningInterface* shootPlanningInterface)
{
    sp = shootPlanningInterface;
}

behTreeReturnEnum cAbstractAction::execute(const std::map<std::string, std::string> &parameters)
{
	throw std::runtime_error("Not implemented");
}

/* \brief Parses string to get Position2D from POI, ball or robotPos.
 *
 */
boost::optional<Position2D> cAbstractAction::getPos2DFromStr(const std::map<std::string, std::string> &parameters, std::string &param)
{
    Position2D retVal;

    auto paramVal = parameters.find(param);
    if (paramVal != parameters.end())
    {
        std::string strParam = paramVal->second;

        // If param is the empty value, return boost::none;
        if (strParam.compare(emptyValue) == 0)
        {
            return boost::none;
        }
        // If param has value 'ball', return ballposition (if possible)
        else if (strParam.compare("ball") == 0)
        {
            ball ball = ballStore::getBall();

            if (ball.isLocationKnown())
            {
                retVal.x = ball.getPosition().x;
                retVal.y = ball.getPosition().y;
                return retVal;
            }
        }
        else if(strParam.compare("lastKnownBallLocation") == 0)
        {
            Point3D ball = ballStore::getBall().getPosition();

        	retVal.x = ball.x;
        	retVal.y = ball.y;
        	return retVal;
        }
        else if (strParam.compare("robot") == 0)
        {
            Position2D myPos;
            cWorldModelInterface::getInstance().getOwnLocation(myPos);

            retVal.x = myPos.x;
            retVal.y = myPos.y;
            return retVal;
        }
        else if (strParam.compare("closestTeammemberIncludeGoalie") == 0)
        {
        	Position2D teammemberPos;
        	cWorldStateFunctions::getInstance().getClosestTeammember(teammemberPos.x, teammemberPos.y, true);

        	retVal.x = teammemberPos.x;
        	retVal.y = teammemberPos.y;
        	return retVal;
        }
        else if (strParam.compare("closestTeammember") == 0)
        {
        	Position2D teammemberPos;
        	cWorldStateFunctions::getInstance().getClosestTeammember(teammemberPos.x, teammemberPos.y, false);

        	retVal.x = teammemberPos.x;
        	retVal.y = teammemberPos.y;
        	return retVal;
        }
        else if (strParam.compare("closestAttacker") == 0)
        {
            double attackerPosX = 0.0;
            double attackerPosY = 0.0;
            cWorldStateFunctions::getInstance().getClosestAttacker(A_FIELD, attackerPosX, attackerPosY);

            retVal.x = attackerPosX;
            retVal.y = attackerPosY;
            return retVal;
        }
        else if (strParam.compare("closestAttackerOnOppHalf") == 0)
        {
            double attackerPosX = 0.0;
            double attackerPosY = 0.0;
            cWorldStateFunctions::getInstance().getClosestAttacker(A_OPP_SIDE, attackerPosX, attackerPosY);

            retVal.x = attackerPosX;
            retVal.y = attackerPosY;
            return retVal;
        }
        else if (strParam.compare("closestAttackerToOppGoal") == 0)
        {
            double attackerPosX = 0.0;
            double attackerPosY = 0.0;
            cWorldStateFunctions::getInstance().getClosestAttackerToOpponentGoal(attackerPosX, attackerPosY);

            retVal.x = attackerPosX;
            retVal.y = attackerPosY;
            return retVal;
        }
        else if (strParam.compare("closestDefender") == 0)
        {
            double defenderPosX = 0.0;
            double defenderPosY = 0.0;
            cWorldStateFunctions::getInstance().getClosestDefender(defenderPosX, defenderPosY);

            retVal.x = defenderPosX;
            retVal.y = defenderPosY;
            return retVal;
        }
        else if (strParam.compare("closestOpponent") == 0)
        {
        	float opponentPosX = 0.0;
        	float opponentPosY = 0.0;
        	cWorldStateFunctions::getInstance().getClosestOpponent(opponentPosX, opponentPosY);

        	retVal.x = opponentPosX;
        	retVal.y = opponentPosY;
        	return retVal;
        }
        else if (strParam.compare("closestOpponentToOwnGoal") == 0)
        {
            Point2D own_goalline_center = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OWN_GOALLINE_CENTER);
        	float opponentPosX = 0.0;
        	float opponentPosY = 0.0;
        	cWorldStateFunctions::getInstance().getClosestOpponentToLocationXY(opponentPosX, opponentPosY, (float) own_goalline_center.x, (float) own_goalline_center.y);

        	retVal.x = opponentPosX;
        	retVal.y = opponentPosY;
        	return retVal;
        }
        else if (strParam.compare("potentialOppAttacker") == 0)
        {
            float opponentAttackerPosX = 0.0;
            float opponentAttackerPosY = 0.0;
            cWorldStateFunctions::getInstance().getPotentialOpponentAttacker(opponentAttackerPosX, opponentAttackerPosY);

            retVal.x = opponentAttackerPosX;
            retVal.y = opponentAttackerPosY;
            return retVal;
        }
        else if( boost::starts_with( strParam, "coord:"))
        {   //expected syntax example:  coord 4.5,6.8
        	std::vector<std::string> splittedString;
        	boost::split( splittedString, strParam, boost::is_any_of(":,"));
        	if ( splittedString.size() != 3)
        	{
        		TRACE("MALFORMED coord parameter encountered: ") << strParam;
        		return boost::none;
        	}

        	try
        	{
        	   retVal.x = boost::lexical_cast<double>( splittedString[1]);
        	}
        	catch (std::exception &e)
			{
        		TRACE("MALFORMED coord parameter encountered for X in: ") << strParam;
        		return boost::none;
			}

        	try
        	{
        	   retVal.y = boost::lexical_cast<double>( splittedString[2]);
        	}
        	catch (std::exception &e)
			{
        		TRACE("MALFORMED coord parameter encountered for Y in: ") << strParam;
        		return boost::none;
			}
        	return retVal;
        }
        else
        {
            try
            {
                Point2D paramInfo = fieldDimensionsStore::getFieldDimensions().getLocation(strParam);
                retVal.x = paramInfo.x;
                retVal.y = paramInfo.y;
            }
            catch (std::exception &e)
            {
                TRACE("MALFORMED location parameter encountered: ") << strParam;
                return boost::none;
            }
            return retVal;
        }
    }
    return boost::none;
}

/* \brief Parses string to get Position2D from POI, ball or robotPos.
 *
 */
boost::optional<Area2D> cAbstractAction::getArea2DFromStr(const std::map<std::string, std::string> &parameters, std::string &param)
{
    Area2D retVal;

    auto paramVal = parameters.find(param);
    if (paramVal != parameters.end())
    {
        std::string strParam = paramVal->second;

        try
        {
            return fieldDimensionsStore::getFieldDimensions().getArea(strParam);
        }
        catch (std::exception &e)
        {

            return boost::none;
        }
    }
    return boost::none;
}


void cAbstractAction::moveTo( double x, double y, double phi, const std::string& motionProfile )
{
    if (!hal)
    {
        hal = new cHALInterface();
        hal->connect();
    }

    if (isPrepareSetPiece())
    {
        hal->disableBallhandlers();
    }
    else
    {
        hal->enableBallhandlers();
    }

    if (!pp)
    {
        pp = new cPathPlanningInterface();
        pp->connect();
    }

    if (!pp->isEnabled())
    {
        pp->enable();
    }

    geometry::Pose2D pose(x, y, phi);
    auto forbidden_areas = getForbiddenAreas();
    pp->moveTo(pose, forbidden_areas, motionProfile);
}


void cAbstractAction::shoot(double shotPower)
{
    if (!sp)
    {
        sp = new cShootPlanningInterface();
        sp->connect();
    }

    if (!sp->isEnabled())
    {
        sp->enable();
    }

    TRACE("Shooting with power: ") << std::to_string(shotPower);
    sp->shoot(shotPower);

    // TODO this is a quick HACK, applied during practice matches against TU/e and VDL
    // Currently, after any pass, the closest robot intercepts the ball 
    // This often turns out to be the one that did the pass... 
    // Add a short sleep (for now) to allow other robots to get the ball instead.
    auto settleTime = configurationStore::getConfiguration().getSettleTimeAfterShooting();
    TRACE("About to sleep for ") << std::to_string(settleTime) << " seconds...";
    sleep(settleTime);
    TRACE("Awake again after shooting");
}

void cAbstractAction::lobShot(const Point2D& target)
{
    if (!sp)
    {
        sp = new cShootPlanningInterface();
        sp->connect();
    }

    if (!sp->isEnabled())
    {
        sp->enable();
    }

    TRACE("Lob Shooting at target: ") << std::to_string(target.x) << ", " << std::to_string(target.y);
    sp->lobShot(target);
}

void cAbstractAction::stop()
{
    /* The "stop" must (1) move to current position and (2) disable motion and the HAL
     * Point (1) is important, otherwise motion will continue persuing its current setpoint
     * until the setpoint watchdog interferes */

    if (!pp)
    {
        pp = new cPathPlanningInterface();
        pp->connect();
    }

    if (pp->isEnabled())
    {
        Position2D ownPosition;
        cWorldModelInterface::getInstance().getOwnLocation( ownPosition );

        geometry::Pose2D pose(ownPosition.x, ownPosition.y, ownPosition.phi);
        pp->moveTo(pose);

        pp->disable();
    }

    if (!hal)
    {
        hal = new cHALInterface();
        hal->connect();
    }

    hal->disableBallhandlers();
}

bool cAbstractAction::positionReached(double x, double y, double phi)
{
    return positionReached(x, y, phi, XYpositionTolerance, PHIpositionTolerance);
}

bool cAbstractAction::positionReached(double x, double y, double phi, double xy_threshold, double phi_threshold)
{
    Position2D myPos;
    cWorldModelInterface::getInstance().getOwnLocation(myPos);

    TRACE(" x: ") << std::to_string(x)
       << " y: " << std::to_string(y)
       << " phi: " << std::to_string(phi)
       << " xy threshold: " << std::to_string(xy_threshold)
       << " phi threshold: " << std::to_string(phi_threshold);

    bool positionReached = ((fabs(myPos.x - x) < xy_threshold) && (fabs(myPos.y - y) < xy_threshold));

    TRACE("position reached: ") << ((positionReached) ? ("yes") : ("no"));

    bool angleReached = (fabs(project_angle_mpi_pi(myPos.phi - phi)) < phi_threshold);

    TRACE("angle reached: ") << ((angleReached) ? ("yes") : ("no"));

    if (positionReached && angleReached)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool cAbstractAction::isCurrentPosValid() const
{
    return rule_p->isCurrentPositionValid();
}

bool cAbstractAction::isTargetPosInsideSafetyBoundaries (const Position2D& targetPos) const
{
    return fieldDimensionsStore::getFieldDimensions().isPositionInSafetyBoundaries(targetPos.x, targetPos.y);
}

std::vector<polygon2D> cAbstractAction::getForbiddenAreas() const
{
    auto forbiddenAreas = rule_p->getForbiddenAreas();

    if (ballStore::getBall().mustBeAvoided())
    {
        auto ballPosition = ballStore::getBall().getPosition();
        polygon2D squareAroundBall;
        squareAroundBall.addPoint(ballPosition.x - 0.25, ballPosition.y - 0.25);
        squareAroundBall.addPoint(ballPosition.x - 0.25, ballPosition.y + 0.25);
        squareAroundBall.addPoint(ballPosition.x + 0.25, ballPosition.y + 0.25);
        squareAroundBall.addPoint(ballPosition.x + 0.25, ballPosition.y - 0.25);


        forbiddenAreas.push_back(squareAroundBall);
    }

    /* Check whether other robot is shooting on goal
     * If so, add forbidden area on those coordinates
     */
    std::vector<polygon2D> forbiddenActionAreas = getForbiddenActionAreas();

    forbiddenAreas.insert(forbiddenAreas.begin(), forbiddenActionAreas.begin(), forbiddenActionAreas.end());

    return forbiddenAreas;
}


Point2D cAbstractAction::getPreferredPartOfGoal() const
{
    // own position
    Position2D myPos;
    cWorldModelInterface::getInstance().getOwnLocation(myPos);

    // some variables that read nice
    auto goalCenter = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALLINE_CENTER);
    auto goalPostLeft = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALPOST_LEFT);
    auto goalPostRight = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALPOST_RIGHT);

    double goalPostShootOffset = 0.35; // the robot should shoot inside the goal area i.o. onto the goalpost
    goalPostLeft.x = goalPostLeft.x + goalPostShootOffset;
    goalPostRight.x = goalPostRight.x - goalPostShootOffset;

    // default aim is opponent goalline center
    auto preferredPartOfGoal = goalCenter;

	if ( isOpponentGoalKeeperInLeftCorner() )
	{
		// Opponent goalkeeper is positioned in left corner, shoot in right corner
		preferredPartOfGoal = goalPostRight;
	}
	else if ( isOpponentGoalKeeperInRightCorner() )
	{
		// Opponent goalkeeper is positioned in right corner, shoot in left corner
		preferredPartOfGoal = goalPostLeft;
	}
	else
	{
		// is the robot positioned on left side of field, shoot right corner
		if (fieldDimensionsStore::getFieldDimensions().isPositionInLeftSide(myPos.x, myPos.y))
		{
			preferredPartOfGoal = goalPostRight;
		}
		else
		{
			// else shoot in left corner
			preferredPartOfGoal = goalPostLeft;
		}
    }
    return preferredPartOfGoal;
}

void cAbstractAction::sendIntention()
{
	try
	{
		if(intention.actionType != actionEnum::INVALID)
		{
			intention.timestamp = getTimeNow();
			intentionSync::getInstance().sendIntention(intention);
		}
	}
	catch(std::exception &e)
	{
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

std::vector<polygon2D> cAbstractAction::getForbiddenActionAreas() const
{
	std::vector<polygon2D> retVal;
	/*
	 * Check shooting on goal
	 */
	robots teammembers = teamMatesStore::getInstance().getTeamMatesIncludingGoalie().getAllRobots();
	for(auto it = teammembers.begin(); it != teammembers.end(); it++)
	{
		intentionStruct intention = teamMatesStore::getInstance().getRobotIntention(it->getNumber());
		if((intention.actionType == actionEnum::AIM_FOR_SHOT_ON_GOAL) &&
				it->getNumber() != getRobotNumber())
		{
			polygon2D shootArea;
			shootArea.addPoint(intention.x - 0.25, intention.y - 0.25);
			shootArea.addPoint(intention.x + 0.25, intention.y + 0.25);
			shootArea.addPoint(it->getPosition().x + 0.25, it->getPosition().y + 0.25);
			shootArea.addPoint(it->getPosition().x - 0.25, it->getPosition().y - 0.25);

			retVal.push_back(shootArea);
		}
	}

	return retVal;
}
