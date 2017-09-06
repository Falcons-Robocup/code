 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * GameSignalSubscriber.h
 *
 *  Created on: May 8, 2016
 *      Author: Diana Koenraadt
 */

#ifndef GAMESIGNALSUBSCRIBER_H
#define GAMESIGNALSUBSCRIBER_H

#include "GameSignalAdapter.h" // For enum BallPossessionType and SignalMode definition
#include <time.h>

/*
* Subscriber interface for signals
* i.e. robot motion, world updates, etc. 
*/
class GameSignalSubscriber
{  
public:
    virtual ~GameSignalSubscriber()
    {
        if (_signalAdapter != NULL)
        {
            _signalAdapter = NULL;
        }
    }

    // Derived classes must call base subscribe first.
    virtual void subscribe(GameSignalAdapter* signalAdapter)
    {
        _signalAdapter = signalAdapter;
    }

    virtual void setSignalMode(SignalMode mode)
    {
        _viewSignalMode = mode;
    }

protected:
    SignalMode _viewSignalMode = WORLD; // Whether to handle integrated world view or per-robot vision view

private:
    GameSignalAdapter* _signalAdapter;

public:
    /* == Robot view  == */
    virtual void onBallPositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel, float confidence) {}; // Ball position according to one robot
    virtual void onBallPossessionChanged(uint8_t senderRobotId, SignalMode mode, BallPossessionType type, uint8_t robotId) {}; // Ball possession according to one robot
    virtual void onOwnTeamPositionChanged(uint8_t senderRobotId, SignalMode mode, uint8_t robotId, PositionVelocity& posvel) {}; // Team member position according to one robot
    virtual void onObstaclePositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel) {}; // Obstacle position according to one robot
    virtual void onForbiddenAreaChanged(ObjectId id, SignalMode signalMode, polygon2D& area) {}; // Obstacle position according to one robot
    virtual void onShootTargetChanged(uint8_t senderRobotId, SignalMode mode, PositionVelocity& posvel, bool aiming) {}; // Shoot target according to one robot
    virtual void onProjectSpeedChanged(ObjectId senderRobotId, SignalMode mode, linepoint2D& speedVector) {}; // Shoot target according to one robot

    /*
    * FPS = vision FPS
    * linePoints = Detected number of white points that constitute a soccer field line
    * age = Vision 'locks' onto the robots own position. Every time it re-establishes its position, lastActive is reset to 0. So, larger is better.
    * lastActive = 
    */
    virtual void onVisionMetaDataChanged(uint8_t senderRobotId, SignalMode mode, float FPS, int linePoints, float age, float lastActive) {};

    /*
    * Pathplanning. 
    * path = list of coordinates, first is first on path, last is the final target on the path
    */
    virtual void onPathPlanningInProgress(uint8_t senderRobotId, std::vector<PositionVelocity>& path) {};

    /*
    * Robot- and team event logging
    */
    virtual void onLog(LogEvent event) {};

    /*
    * Free form value logging
    */
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, float value) {};
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, bool value) {};
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, std::string value) {};
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, std::vector<std::string> value) {};

    /*
    * Match analytics
    * logElapsedTime: The time elapsed during this logging (t=0 is the start of the logfile)
    * actualTime: The time on the clock at the current match state (e.g. 17:23:46.889), can be related to on-robot tracing
    */
    virtual void onClockTick(double logElapsedTime, double actualTime) {}; 
};

#endif // GAMESIGNALSUBSCRIBER_H
