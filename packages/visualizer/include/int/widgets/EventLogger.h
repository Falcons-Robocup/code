 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * EventLogger.h
 *
 *  Created on: August 1, 2016
 *      Author: Diana Koenraadt
 */

#ifndef EVENTLOGGER_H
#define EVENTLOGGER_H

#include <QtGui>

#include <boost/format.hpp>

#include "timeConvert.hpp"

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
                timeString = timeToString(event.timeStamp);
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
