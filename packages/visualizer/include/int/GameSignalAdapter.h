 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef GAMESIGNALADAPTER_H
#define GAMESIGNALADAPTER_H

#include <QtGui>

#include "types/PositionVelocity.h"
#include "types/LogEvent.h"
#include "types/ObjectId.h"
#include "polygon2D.hpp"
#include "rtdbStructs.hpp"

enum BallPossessionType : uint8_t
{
    TYPE_INVALID = 0,
    TYPE_FIELD = 1,
    TYPE_TEAMMEMBER = 2,
    TYPE_OPPONENT = 3
};

enum SignalMode
{
    WORLD,
    VISION,
    PATHPLANNING,
    TEAMPLAY,
    GAUSSIAN_WORLD,
    GAUSSIAN_MEASUREMENTS
};

enum DataType
{
    UNDEFINED,
    ROBOTPOSITION,
    BALLPOSITION,
    BALLPOSSESSION,
    OBSTACLEPOSITION,
    FORBIDDENAREA,
    SHOOTTARGET,
    PROJECTSPEEDVECTOR,
    PATHPLANNINGPROGRESS,
    WORLDMODEL_LOCAL
};

enum CameraType
{
    OMNIVISION,
    FRONTVISION,
    MULTIVISION,
};

/*
* Pure virtual interface for signals
* i.e. robot motion, world updates, etc. 
* DKPJ: Note, do not expose any Falcons-specific types anywhere but in the Adapter implementation.
* Goal is to have visualizer as complete standalone application that other teams merely need to write a new adapter for.
*/
class GameSignalAdapter : public QObject
{
    Q_OBJECT

Q_SIGNALS:
    /* == Robot view  == */
    void signalBallPositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel, float confidence, float age, CameraType camera); // Ball position according to one robot, also used for vision measurements
    void signalBallPossessionChanged(uint8_t senderRobotId, SignalMode mode, BallPossessionType type, uint8_t robotId); // Ball possession according to one robot
    void signalOwnTeamPositionChanged(uint8_t senderRobotId, SignalMode mode, uint8_t robotId, PositionVelocity& posvel); // Team member position according to one robot
    void signalRobotStatusChanged(uint8_t senderRobotId, SignalMode mode, uint8_t robotId, int status); // indicator for robot status
    void signalObstaclePositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel); // Obstacle position according to one robot
    void signalForbiddenAreaChanged(ObjectId _t1, SignalMode _t2, polygon2D & _t3); // Forbidden area according to one robot
    void signalShootTargetChanged(uint8_t id, SignalMode mode, PositionVelocity& posvel, bool aiming);
    void signalProjectSpeedChanged(ObjectId id, SignalMode mode, linepoint2D& speedVector);
    void signalClearRobot(uint8_t robotId);
    void signalGaussianObstaclesUpdate(uint8_t senderRobotId, SignalMode mode, T_DIAG_WORLDMODEL_LOCAL& worldmodel_local);
    /*
    * Time since start of data log, used in renderer for hiding objects
    */
    void signalClearAll();
    void signalElapsedTimeChanged(double logElapsedTime);
    
    /*
    * Pathplanning. 
    * path = list of coordinates, first is first on path, last is the final target on the path
    */
    void signalPathPlanningInProgress(uint8_t senderRobotId, std::vector<PositionVelocity>& path);

    /*
    * Robot- and team event logging
    */
    void signalLog(LogEvent event); 

    /*
    * Free form text logging
    */
    void signalValue(uint8_t senderRobotId, std::string category, std::string key, float value);
    void signalValue(uint8_t senderRobotId, std::string category, std::string key, bool value);
    void signalValue(uint8_t senderRobotId, std::string category, std::string key, std::string value);
    void signalValue(uint8_t senderRobotId, std::string category, std::string key, std::vector<std::string> value);
    /*
    * Match analytics
    * logElapsedTime: The time elapsed during this logging (t=0 is the start of the logfile)
    * actualTime: The time on the clock at the current match state (e.g. 17:23:46.889), can be related to on-robot tracing
    */
    void signalClockTick(double logElapsedTime, double actualTime);
    // TODO: signalMatchTick: The time elapsed during match (t=0 at refbox half start)

    void signalRefBoxCommand(uint8_t senderRobotId, std::string command);
    void signalRefBoxCommandTime(uint8_t senderRobotId, double commandTime);
    void signalGoal(uint8_t senderRobotId, int goals);
    void signalPhase(uint8_t senderRobotId, int phase);
    void signalOutOfPlay(uint8_t senderRobotId, bool outofplay);
};

#endif // GAMESIGNALADAPTER_H
