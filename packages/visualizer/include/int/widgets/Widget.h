// Copyright 2016-2022 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Widget.h
 *
 *  Created on: June 30, 2016
 *      Author: Diana Koenraadt
 */

#ifndef WIDGET_H
#define WIDGET_H

#include <QtGui>
#include <qdialog.h>

// Internal:
#include "int/TeamRobotSelection.h"
#include "int/GameSignalSubscriber.h"
#include "int/adapters/GameSignalAdapter.h"

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

    // Called after widget settings were changed
    virtual void reloadSettings() {};

private:
    TWidgetGameSignalSubscriber* _signalSubscriber; 
};

#endif // WIDGET_H
