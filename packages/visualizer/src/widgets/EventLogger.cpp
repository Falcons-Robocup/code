 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * EventLogger.cpp
 *
 *  Created on: November 4, 2016
 *      Author: Diana Koenraadt
 */

// Internal:
#include "int/widgets/EventLogger.h"

EventLogger::EventLogger(QWidget* parent)
      : QWidget(parent),
        Widget()
{
    _autoScrollEnabled = false;
    _maxLogLevelToLog = LogLevelList[0];
    // several minor TODO items:
    // * fix scrolling: always show most recent events, allow user to scroll up to see history
    // * fix time sorting: due to wifi or time offsets, it can happen that events are not consecutive in time
    // * in playback mode: clear event window when slider is moved backwards
    // * in playback mode: if speedup is large, verify that no events are lost

    QVBoxLayout* layoutV = new QVBoxLayout(this);

    QHBoxLayout* layoutH1 = new QHBoxLayout(this);
    layoutV->addLayout(layoutH1);
    QCheckBox* checkBoxAutoScroll = new QCheckBox(this);
    checkBoxAutoScroll->setChecked(true);
    layoutH1->addWidget(checkBoxAutoScroll, 0, Qt::AlignLeft);
    QLabel* autoScrollLabel = new QLabel("Auto scroll", this);
    layoutH1->addWidget(autoScrollLabel, 3, Qt::AlignLeft);

    _textWidget = new QPlainTextEdit(this);
    _textWidget->setReadOnly(true);
    _textWidget->setCenterOnScroll(true);
    layoutV->addWidget(_textWidget);

    QHBoxLayout* layoutH2 = new QHBoxLayout(this);
    layoutV->addLayout(layoutH2);

    QLabel* logLevelLabel = new QLabel("Min. log level", this);
    layoutH2->addWidget(logLevelLabel);
    QComboBox* logLevelComboBox = new QComboBox(this);
    layoutH2->addWidget(logLevelComboBox);
    QPushButton* clearButton = new QPushButton("Clear", this);
    layoutH2->addWidget(clearButton);

    // Allow toggle of autoscrolling in eventlog
    connect(checkBoxAutoScroll, SIGNAL(stateChanged(int)), this, SLOT(autoScroll(int)));
    _textWidget->setCenterOnScroll(checkBoxAutoScroll->isChecked());

    // Allow toggle of event log levels
    for (auto level : LogLevelList)
    {
        logLevelComboBox->addItem(ToString(level));
    }
    connect(logLevelComboBox, SIGNAL(activated(int)), this, SLOT(logLevelSelectionChanged(int)));
    logLevelComboBox->setCurrentIndex(0);

    // Connect clear button
    connect(clearButton, SIGNAL(pressed()), this, SLOT(clear()));

}

void EventLogger::log(LogEvent event)
{
    Message* msg = new Message(event);

    std::string formattedMessage = msg->getFormattedMessage();
    // event generation has been reworked - no more filtering needed
    _logMessagesLifetime[formattedMessage] = 0;
    _messages.push_back(msg);
    write(*msg);
}

void EventLogger::write(Message message)
{
    if (_maxLogLevelToLog <= message.event.type)
    {
        _textWidget->appendHtml(message.getFormattedMessage().c_str());

        if (_autoScrollEnabled)
        {
            _textWidget->ensureCursorVisible();
        }
    }
}

void EventLogger::autoScroll(int checkBoxState)
{
    _autoScrollEnabled = checkBoxState == Qt::Checked; 
}

void EventLogger::logLevelSelectionChanged(int index)
{
    LogLevel newLogLevel = static_cast<LogLevel>(index);

    if (_maxLogLevelToLog == newLogLevel)
    {
        return;
    }
    
    _maxLogLevelToLog = newLogLevel;

    // Clear the log and reiterate over all messages to show them.
    clear();
    for (int i = 0; i < (int)_messages.size(); ++i)
    {
        Message m = *(_messages[i]);
        write(m);
    }
}

void EventLogger::clear()
{
    _textWidget->clear();
    _logMessagesLifetime.clear(); 
}
