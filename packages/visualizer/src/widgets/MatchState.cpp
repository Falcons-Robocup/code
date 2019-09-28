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
 *  Created on: May 09, 2019
 *      Author: Lu Dai
 */

#include <QString>
#include <string>

// Internal:
#include "int/widgets/MatchState.h"

MatchState::MatchState(QWidget* parent)
      : Widget()
{
    setSegmentStyle(Filled);
    setDigitCount(12);

    QGridLayout *matchStateLayout = new QGridLayout(this);

    clear();

    QLabel *phaseLabel = new QLabel(QStringLiteral("Match Phase : "));
    phaseLabel->setStyleSheet("font: 10pt; color: white; ");
    _phase = new QLabel();
    _phase->setStyleSheet("font: 10pt; color: white; ");

    QLabel *lastCommandLabel = new QLabel(QStringLiteral("Last Command : "));
    lastCommandLabel->setStyleSheet("font: 10pt; color: white; ");
    lastCommandLabel->resize(this->width()/3, this->height());
    _lastCommand = new QLabel();
    _lastCommand->setStyleSheet("font: 10pt; color: white; ");

    QLabel *goalLabel = new QLabel(QStringLiteral("Goals : "));
    goalLabel->setStyleSheet("font: 10pt; color: white; ");
    _goals = new QLabel();
    _goals->setStyleSheet("font: 10pt; color: white; ");

    QLabel *lastCommandTimeLabel = new QLabel(QStringLiteral("Last Command Time : "));
    lastCommandTimeLabel->setStyleSheet("font: 10pt; color: white; ");
    lastCommandTimeLabel->resize(this->width()/3, this->height());
    _lastCommandTime = new QLabel();
    _lastCommandTime->setStyleSheet("font: 10pt; color: white; ");

    matchStateLayout->addWidget(phaseLabel, 0, 0);
    matchStateLayout->addWidget(_phase, 0, 1);
    matchStateLayout->addWidget(goalLabel,1,0);
    matchStateLayout->addWidget(_goals,1,1);
    matchStateLayout->addWidget(lastCommandLabel, 2, 0);
    matchStateLayout->addWidget(_lastCommand, 2, 1);
    matchStateLayout->addWidget(lastCommandTimeLabel, 3, 0);
    matchStateLayout->addWidget(_lastCommandTime, 3, 1);
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
