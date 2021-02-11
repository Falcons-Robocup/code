// Copyright 2019 ludai (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

