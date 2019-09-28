 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * MatchState.h
 *
 *  Created on: May 9, 2019
 *      Author: Lu Dai
 */

#ifndef MATCHSTATE_H
#define MATCHSTATE_H

#include <QtGui>
#include <QLCDNumber>
#include "FalconsRtDB2.hpp" // for rtime
#include <qgridlayout.h>
#include <qcheckbox.h>
#include <qlabel.h>
#include <qfont.h>
#include <QWidget>
// Internal:
#include "int/widgets/Widget.h"


class GameSignalAdapter;
class MatchState;

class MatchStateGameSignalSubscriber : public QObject, public WidgetGameSignalSubscriber<MatchState>
{
    Q_OBJECT
public:
    using WidgetGameSignalSubscriber::WidgetGameSignalSubscriber;
    virtual void subscribe(GameSignalAdapter* gameSignalAdapter) override;

public Q_SLOTS:
    virtual void onCommandChanged(uint8_t senderRobotId, std::string command) override;
    virtual void onCommandTimeChanged(uint8_t senderRobotId, double commandTime) override;
    virtual void onGoal(uint8_t senderRobotId, int goals) override;
    virtual void onPhaseChanged(uint8_t senderRobotId, int phase) override;
};

/*
* Match State class
*/

class MatchState : public QLCDNumber, public Widget<MatchStateGameSignalSubscriber, MatchState>
{
    Q_OBJECT
friend MatchStateGameSignalSubscriber;

public:
    MatchState(QWidget* parent = 0);

private:
    QLabel *_phase;
    QLabel *_lastCommand;
    QLabel *_lastCommandTime;
    QLabel *_goals;
    void clear();
    void setCommand(QString command);
    void setCommandTime(QString commandTime);
    void setGoal(QString goals);
    void setPhase(QString phase);
};
/*
* Class that handles subscriber data for the Match State
*/
#endif // MATCHSTATE_H

