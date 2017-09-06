 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * FieldWidgetGameSignalSubscriber.h
 *
 *  Created on: May 15, 2016
 *      Author: Diana Koenraadt
 */

// Internal:
#include "int/widgets/Field/FieldWidget3D.h"

#include <boost/format.hpp>

// Falcons shared code:
#include "tracer.hpp"

void FieldWidgetGameSignalSubscriber::setSignalMode(SignalMode mode) 
{
    GameSignalSubscriber::setSignalMode(mode);

    _widget->clear();
}

void FieldWidgetGameSignalSubscriber::subscribe(GameSignalAdapter* gameSignalAdapter)
{
    WidgetGameSignalSubscriber::subscribe(gameSignalAdapter);

    QObject::connect(gameSignalAdapter, SIGNAL(signalBallPositionChanged(ObjectId, SignalMode, PositionVelocity&, float)), this, SLOT(onBallPositionChanged(ObjectId, SignalMode, PositionVelocity&, float)));

    QObject::connect(gameSignalAdapter, SIGNAL(signalObstaclePositionChanged(ObjectId, SignalMode, PositionVelocity&)), this, SLOT(onObstaclePositionChanged(ObjectId, SignalMode, PositionVelocity&)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalForbiddenAreaChanged(ObjectId, SignalMode, polygon2D&)), this, SLOT(onForbiddenAreaChanged(ObjectId, SignalMode, polygon2D&)));

    QObject::connect(gameSignalAdapter, SIGNAL(signalOwnTeamPositionChanged(uint8_t, SignalMode, uint8_t, PositionVelocity&)), this, SLOT(onOwnTeamPositionChanged(uint8_t, SignalMode, uint8_t, PositionVelocity&)));

    QObject::connect(gameSignalAdapter, SIGNAL(signalElapsedTimeChanged(double)), this, SLOT(onElapsedTimeChanged(double)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalPathPlanningInProgress(uint8_t, std::vector<PositionVelocity>&)), this, SLOT(onPathPlanningInProgress(uint8_t, std::vector<PositionVelocity>&)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalShootTargetChanged(uint8_t, SignalMode, PositionVelocity&, bool)), this, SLOT(onShootTargetChanged(uint8_t, SignalMode, PositionVelocity&, bool)));
    QObject::connect(gameSignalAdapter, SIGNAL(signalProjectSpeedChanged(ObjectId, SignalMode, linepoint2D&)), this, SLOT(onProjectSpeedChanged(ObjectId, SignalMode, linepoint2D&)));
}

void FieldWidgetGameSignalSubscriber::onTeamModeChanged()
{
    _widget->clear();
}

void FieldWidgetGameSignalSubscriber::onRobotModeChanged()
{
    _widget->clear();
}

void FieldWidgetGameSignalSubscriber::onElapsedTimeChanged(double t)
{
    _widget->setLogTimeStamp(t);
    // detect if time slider is pulled back
    // all actors should be hidden immediately and eventlog cleared
    static double prevT = 0;
    if (t < prevT)
    {
        _widget->clear();
    }
    prevT = t;
}

/* 
* ========================================
*           Robot view
* ======================================== 
*/
bool FieldWidgetGameSignalSubscriber::displayDataFilter(const uint8_t& robotID, const SignalMode& signalMode, const DataType &dataType)
{
    // When returned true, onXXXChanged data is drawn.
    bool result = false;
    if (_viewMode == ROBOT && _robotModeId == robotID && _viewSignalMode == signalMode)
    {
        result = true;
    }
    else if (_viewMode == TEAM)
    {
        // For signalMode PathPlanning, we want to see WorldModel + PathPlanning.
        if (_viewSignalMode == SignalMode::PATHPLANNING)
        {
            if (signalMode == SignalMode::PATHPLANNING || signalMode == SignalMode::WORLD)
            {
                result = true;
            }
        }
        // For signalMode Teamplay, we want to see WorldModel + Teamplay.
        else if (_viewSignalMode == SignalMode::TEAMPLAY)
        {
            if (signalMode == SignalMode::TEAMPLAY || signalMode == SignalMode::WORLD)
            {
                result = true;
            }
        }
        // For all other signalModes, we only want the active signalMode.
        else if (_viewSignalMode == signalMode)
        {
            // Team data is handled as robotID 0
            if (robotID == 0)
            {
                result = true;
            }
        }
    }
    // debug tracing
    //TRACE("displayDataFilter(%d, %d, %d) = %d", (int)robotID, (int)signalMode, (int)dataType, (int)result);
    return result;
}

void FieldWidgetGameSignalSubscriber::onBallPositionChanged(ObjectId id, SignalMode signalMode, PositionVelocity& posvel, float confidence)
{
    // Only process signal for the robot we're interested in. 
    // If we're interested in the world view and we're in vision mode, aggregate *all* robot's views.
    if (displayDataFilter(id.robotID, signalMode, DataType::BALLPOSITION))
    {
        TRACE(boost::str(boost::format("FieldWidgetGameSignalSubscriber::onBallPositionChanged(%1%, %2%)") % posvel.x % posvel.y).c_str());
        _widget->setBallPosition(posvel, id);
        // TODO ball color gradient (bright yellow when confidence >= 0.7, orange when confidence <= 0.2)
        //_widget->setBallConfidenceColor(confidence);
    }
}

void FieldWidgetGameSignalSubscriber::onBallPossessionChanged(uint8_t senderRobotId, SignalMode signalMode, BallPossessionType type, uint8_t robotId)
{
    // Only process signal for the robot we're interested in. 
    // If we're interested in the world view and we're in vision mode, aggregate *all* robot's views.
    if (displayDataFilter(senderRobotId, signalMode, DataType::BALLPOSSESSION))
    {
        TRACE("FieldWidgetGameSignalSubscriber::onBallPossessionChanged()");
        switch (type)
        {
            case TYPE_INVALID: 
                break;
            case TYPE_FIELD: 
                break;
            case TYPE_OPPONENT: 
                break;
            case TYPE_TEAMMEMBER: 
                break;
            default: break;
        }
    }
}

void FieldWidgetGameSignalSubscriber::onObstaclePositionChanged(ObjectId id, SignalMode signalMode, PositionVelocity& posvel)
{
    // Only process signal for the robot we're interested in. 
    // If we're interested in the team view and we're in vision mode, aggregate *all* robot's views.
    if (displayDataFilter(id.robotID, signalMode, DataType::OBSTACLEPOSITION))
    {
        //TRACE(boost::str(boost::format("FieldWidgetGameSignalSubscriber::onObstaclePositionChanged(%1%, %2%)") % posvel.x % posvel.y).c_str());
        _widget->setObstaclePosition(posvel, id);
    }
}

void FieldWidgetGameSignalSubscriber::onForbiddenAreaChanged(ObjectId id, SignalMode signalMode, polygon2D& area)
{
	// Only process signal for the robot we're interested in.
    // If we're interested in the team view and we're in vision mode, aggregate *all* robot's views.
    if (displayDataFilter(id.robotID, signalMode, DataType::OBSTACLEPOSITION))
    {
        //TRACE(boost::str(boost::format("FieldWidgetGameSignalSubscriber::onForbiddenAreaChanged(%1%, %2%)") % posvel.x % posvel.y).c_str());
        _widget->setForbiddenAreaPosition(area, id);
    }
}

void FieldWidgetGameSignalSubscriber::onOwnTeamPositionChanged(uint8_t senderRobotId, SignalMode signalMode, uint8_t robotId, PositionVelocity& posvel)
{
    // Only process signal for the robot we're interested in. 
    // If we're interested in the world view and we're in vision mode, aggregate *all* robot's views.
    if (displayDataFilter(senderRobotId, signalMode, DataType::ROBOTPOSITION))
    {
        TRACE(boost::str(boost::format("FieldWidgetGameSignalSubscriber::onOwnTeamPositionChanged(%1%, %2%)") % posvel.x % posvel.y).c_str());
        _widget->getTeamRobot(robotId)->setPosition(posvel);
        _widget->getTeamRobot(robotId)->VisibilityOn();
    }
}

void FieldWidgetGameSignalSubscriber::onPathPlanningInProgress(uint8_t senderRobotId, std::vector<PositionVelocity>& path) 
{
    // Only process signal for the robot we're interested in. 
    // If we're interested in the world view and we're in vision mode, aggregate *all* robot's views.
    if ((_viewMode == ROBOT && _robotModeId == senderRobotId) || (_viewMode == TEAM))
    {
        TRACE(boost::str(boost::format("FieldWidgetGameSignalSubscriber::onPathPlanningInProgress(%1%)") % senderRobotId).c_str());
        _widget->getTeamRobot(senderRobotId)->setPath(path);
    }
}

void FieldWidgetGameSignalSubscriber::onShootTargetChanged(uint8_t senderRobotId, SignalMode signalMode, PositionVelocity& posvel, bool aiming)
{
    if (displayDataFilter(senderRobotId, signalMode, DataType::SHOOTTARGET))
    {
        if (aiming)
        {
            TRACE(boost::str(boost::format("FieldWidgetGameSignalSubscriber::onShootTargetChanged(%1%, %2%)") % posvel.x % posvel.y).c_str());
            _widget->setShootTargetPosition(senderRobotId, posvel);
        }
        else
        {
            TRACE("FieldWidgetGameSignalSubscriber::onShootTargetChanged: no shoot target");
            _widget->hideShootTarget(senderRobotId);
        }
    }
    else
    {
        _widget->hideShootTarget(senderRobotId);
    }
}

void FieldWidgetGameSignalSubscriber::onProjectSpeedChanged(ObjectId id, SignalMode signalMode, linepoint2D& line)
{
	// Only process signal for the robot we're interested in.
    // If we're interested in the team view and we're in vision mode, aggregate *all* robot's views.
    if (displayDataFilter(id.robotID, signalMode, DataType::PROJECTSPEEDVECTOR))
    {
        //TRACE(boost::str(boost::format("FieldWidgetGameSignalSubscriber::onForbiddenAreaChanged(%1%, %2%)") % posvel.x % posvel.y).c_str());
        _widget->setProjectSpeedPosition(line, id);
    }
}
