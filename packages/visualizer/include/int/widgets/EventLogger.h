// Copyright 2016-2019 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * EventLogger.h
 *
 *  Created on: August 1, 2016
 *      Author: Diana Koenraadt
 */

#ifndef EVENTLOGGER_H
#define EVENTLOGGER_H

#include <QtGui>
#include <qplaintextedit.h>

#include <boost/format.hpp>
#include "FalconsRtDB2.hpp" // for rtime

// Internal:
#include "int/widgets/Widget.h"
#include "int/TeamRobotSelection.h"

class GameSignalAdapter;
class EventLoggerGameSignalSubscriber;

/*
* Event logging class
*/
class EventLogger : public QWidget, public Widget<EventLoggerGameSignalSubscriber, EventLogger>
{
        Q_OBJECT
friend EventLoggerGameSignalSubscriber;

public:
    EventLogger(QWidget* parent = 0);
    void log(LogEvent);

private:
    QPlainTextEdit* _textWidget;
    bool _autoScrollEnabled;
    LogLevel _maxLogLevelToLog;
    std::map<std::string, uint8_t> _logMessagesLifetime;
    QTimer* _lifetimeTimer;

    struct Message
    {
        Message(LogEvent e)
        {
            event = e;
        }

        LogEvent event;

        std::string getFormattedMessage()
        {
            boost::format format_message("r%1% - %2%");
            // TODO: optionally show details such as originating file+line? maybe that is a bit too much, 
            // as it is only interesting during development-specific, and if needed to inspect post-match, 
            // one can also look into tracing
            std::string data = boost::str(format_message % (int)event.robotId % event.message);
            // prepend timestamp, ignore the date part (useless waste of pixels)
            std::string timeString;
            if (event.timeStamp > 0)
            {
                timeString = rtime().fromDouble(event.timeStamp).toStr();
                timeString = timeString.substr(11, 12) + " - ";
            }
            else
            {
                // not filled in?!
                timeString = "            - ";
            }
            data = timeString + data;
            // coloring
            std::string msg = data;
            switch (event.type)
            {
                case INFO: break;
                case WARNING:
                    msg = "<font color=\"yellow\">" + data + "</font>"; 
                    break;
                case ERROR:
                    msg = "<font color=\"red\">" + data + "</font>";
                    break;
                default: break;
            }
            // fixed-width font
            msg = "<pre>" + msg + "</pre>";
            return msg;
        }
    };

    std::vector<Message*> _messages; // Buffer of all messages

    void write(Message message);

public Q_SLOTS:
    void autoScroll(int checkBoxState);
    void logLevelSelectionChanged(int index);
    void clear();
};

/*
* Class that handles subscriber data for the Event Logger
*/
class EventLoggerGameSignalSubscriber : public QObject, public WidgetGameSignalSubscriber<EventLogger>
{
    Q_OBJECT
public:
    using WidgetGameSignalSubscriber::WidgetGameSignalSubscriber;
    virtual void subscribe(GameSignalAdapter* gameSignalAdapter) override;

public Q_SLOTS:
    virtual void onLog(LogEvent event) override;
};

#endif // EVENTLOGGER_H
