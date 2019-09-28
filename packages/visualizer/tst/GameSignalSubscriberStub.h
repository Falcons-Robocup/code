 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * GameSignalSubscriberStub.h
 *
 *  Created on: May 8, 2016
 *      Author: Diana Koenraadt
 */

#ifndef GAMESIGNALSUBSCRIBERSTUB_H
#define GAMESIGNALSUBSCRIBERSTUB_H

#include <QObject>

#include <vector>

#include "int/GameSignalAdapter.h"
#include "int/GameSignalSubscriber.h"

/*
* Helper class that subscribes to all game signals and keeps a list of string representations of the received signals
* Note: Rounds to 2 decimal places
*/
class GameSignalSubscriberStub : public QObject, public GameSignalSubscriber
{
    Q_OBJECT
public:
    GameSignalSubscriberStub(GameSignalAdapter* adapter);
    ~GameSignalSubscriberStub();
    void subscribeBallPositionChanged();
    void subscribeBallPossessionChanged();
    void subscribeObstaclePositionChanged();
    void subscribeOwnTeamPositionChanged();
    void subscribeValue();
    void subscribeTime();
    void subscribePathPlanning();

    std::vector<std::string> receivedMessages;

private:
    GameSignalAdapter* signalAdapter;
    

public Q_SLOTS:
    virtual void onBallPositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel, float confidence, float age, CameraType camType) override; // Ball position according to one robot
    virtual void onBallPossessionChanged(uint8_t senderRobotId, SignalMode mode, BallPossessionType type, uint8_t robotId) override; // Ball possession according to one robot
    virtual void onOwnTeamPositionChanged(uint8_t senderRobotId, SignalMode mode, uint8_t robotId, PositionVelocity& posvel) override; // Team member position according to one robot
    virtual void onObstaclePositionChanged(ObjectId id, SignalMode mode, PositionVelocity& posvel) override; // Obstacle position according to one robot

    virtual void onLog(LogEvent event) override;
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, float value) override;
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, bool value) override;
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, std::string value) override;

    virtual void onClockTick(double elapsedTime, double actualTime) override;

    virtual void onPathPlanningInProgress(uint8_t senderRobotId, std::vector<PositionVelocity>& path) override;
};

#endif // GAMESIGNALSUBSCRIBERSTUB_H
