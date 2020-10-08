 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * arbiter.hpp
 *
 *  Created on: Jan 10, 2019
 *      Author: Coen Tempelaars
 */

#ifndef ARBITER_HPP_
#define ARBITER_HPP_

#include "arbiterGameData.hpp"
#include "gameState.hpp"
#include "judgement.hpp"
#include "preparingController.hpp"
#include "runningController.hpp"
#include "stoppingController.hpp"
#include "stoppedController.hpp"

class Arbiter {
public:
    Arbiter();

    void initialize();
    GameData control(const GameData&, const float simulationPeriod);

    GameState getGameState() const;

protected:
    AbstractRefBoxAdapter* _refBoxAdapter;

private:
    PreparingController _preparingController;
    RunningController _runningController;
    StoppingController _stoppingController;
    StoppedController _stoppedController;

    void goalSignalHandler(const TeamID&);
    void preparingSignalHandler();
    void preparedSignalHandler();
    void stoppingSignalHandler();
    void stoppingSignalHandler(const Judgement&);
    void stoppedSignalHandler();

    ArbiterGameData _arbiterGameData;
    GameState _gameState;
    Judgement _lastJudgement;
    float _secondsSinceLastTransition;
};

#endif /* ARBITER_HPP_ */
