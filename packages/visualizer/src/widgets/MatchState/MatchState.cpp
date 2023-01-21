// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * MatchState.h
 *
 *  Created on: May 09, 2019
 *      Author: Lu Dai
 */

#include <QString>
#include <string>

// Internal:
#include "int/widgets/MatchState/MatchState.h"

#include <stdio.h>

MatchState::MatchState(QWidget* parent)
      : Widget()
{
    setSegmentStyle(Filled);
    setDigitCount(12);

    QGridLayout *matchStateLayout = new QGridLayout(this);

    clear();

    QLabel *phaseLabel = new QLabel(QStringLiteral("Match Phase : "));
    // phaseLabel->setStyleSheet("font: 10pt; color: white; ");
    _phase = new QLabel();
    // _phase->setStyleSheet("font: 10pt; color: white; ");

    QLabel *lastCommandLabel = new QLabel(QStringLiteral("Last Command : "));
    // lastCommandLabel->setStyleSheet("font: 10pt; color: white; ");
    // lastCommandLabel->resize(this->width()/3, this->height());
    _lastCommand = new QLabel();
    // _lastCommand->setStyleSheet("font: 10pt; color: white; ");

    QLabel *goalLabel = new QLabel(QStringLiteral("Goals : "));
    // goalLabel->setStyleSheet("font: 10pt; color: white; ");
    _goals = new QLabel();
    // _goals->setStyleSheet("font: 10pt; color: white; ");

    QLabel *lastCommandTimeLabel = new QLabel(QStringLiteral("Last Command Time : "));
    // lastCommandTimeLabel->setStyleSheet("font: 10pt; color: white; ");
    // lastCommandTimeLabel->resize(this->width()/3, this->height());
    _lastCommandTime = new QLabel();
    // _lastCommandTime->setStyleSheet("font: 10pt; color: white; ");

    matchStateLayout->addWidget(phaseLabel, 0, 0);
    matchStateLayout->addWidget(_phase, 0, 1);
    matchStateLayout->addWidget(goalLabel,1,0);
    matchStateLayout->addWidget(_goals,1,1);
    matchStateLayout->addWidget(lastCommandLabel, 2, 0);
    matchStateLayout->addWidget(_lastCommand, 2, 1);
    matchStateLayout->addWidget(lastCommandTimeLabel, 3, 0);
    matchStateLayout->addWidget(_lastCommandTime, 3, 1);

    std::cout << minimumSizeHint().height() << std::endl;
    setFixedHeight(minimumSizeHint().height());
}

void MatchState::setPhase(QString phase)
{
    _phase->setText(phase);
}

void MatchState::setCommand(QString command)
{
    _lastCommand->setText(command);
}

void MatchState::setCommandTime(QString commandTime)
{
    _lastCommandTime->setText(commandTime);
}

void MatchState::setGoal(QString goals)
{
    _goals->setText(goals);
}

void MatchState::clear()
{
    display("");
}
