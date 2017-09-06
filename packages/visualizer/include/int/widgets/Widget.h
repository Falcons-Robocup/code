 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Widget.h
 *
 *  Created on: June 30, 2016
 *      Author: Diana Koenraadt
 */

#ifndef WIDGET_H
#define WIDGET_H

#include <QtGui>

// Internal:
#include "int/TeamRobotSelection.h"
#include "int/GameSignalSubscriber.h"
#include "int/GameSignalAdapter.h"

template <class TWidgetGameSignalSubscriber, class TWidget, class TSettingsDialog = QDialog>
class Widget;

/*
* Class that handles subscriber data for the widget
*/
template <class TWidget>
class WidgetGameSignalSubscriber : public GameSignalSubscriber, public TeamRobotSelection
{
public:
    WidgetGameSignalSubscriber(TWidget* widget)
    {
        _widget = widget;
    }

    virtual ~WidgetGameSignalSubscriber()
    {
        _widget = NULL;
    }

protected:
    TWidget* _widget;
};

class WidgetBase
{
public:
    virtual GameSignalSubscriber* getSignalSubscriber() = 0;
    virtual TeamRobotSelection* getTeamRobotSelection() = 0;

    virtual void saveState() {};
    virtual void restoreState() {};
};

/*
* Widget class
*/
template <class TWidgetGameSignalSubscriber, class TWidget, class TSettingsDialog> // Default template parameter TSettingsDialog = QDialog declared earlier.
class Widget : public WidgetBase
{
friend TWidgetGameSignalSubscriber;

public:
    Widget()
      : _signalSubscriber(NULL)
    {
        _signalSubscriber = new TWidgetGameSignalSubscriber(static_cast<TWidget*>(this));
        reloadSettings();
    }

    virtual ~Widget()
    {
        if (_signalSubscriber != NULL)
        {
            delete _signalSubscriber;
            _signalSubscriber = NULL;
        }
    }

    virtual GameSignalSubscriber* getSignalSubscriber() override
    {
        return _signalSubscriber;
    }

    virtual TeamRobotSelection* getTeamRobotSelection() override
    {
        return _signalSubscriber;
    }

    // Show dialog to change widget settings
    virtual void showSettingsDialog()
    {
        TSettingsDialog dialog;
        QDialog::DialogCode result = (QDialog::DialogCode)dialog.exec();
        if (result == QDialog::Accepted)
        {
            reloadSettings();
        }
    }

protected:
    // Called after widget settings were changed
    virtual void reloadSettings() {};

private:
    TWidgetGameSignalSubscriber* _signalSubscriber; 
};

#endif // WIDGET_H
