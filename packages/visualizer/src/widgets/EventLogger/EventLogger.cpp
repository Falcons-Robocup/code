// Copyright 2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * EventLogger.cpp
 *
 *  Created on: November 4, 2016
 *      Author: Diana Koenraadt
 */

#include <qboxlayout.h>
#include <qcheckbox.h>
#include <qlabel.h>
#include <qcombobox.h>
#include <qpushbutton.h>

// Internal:
#include "int/widgets/EventLogger/EventLogger.h"

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
