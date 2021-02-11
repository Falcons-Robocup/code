// Copyright 2015-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <ui_MainWindow.h>

#include <QtGui>

#include <iostream>
#include <string>

// Internal:
#include "int/GameSignalAdapter.h"
#include "int/RefboxConfigAdapter.h"
#include "int/PlaybackControl.h"


using namespace std;

namespace Visualizer
{
    class MainWindow : public QMainWindow, public Ui::MainWindow
    {
        Q_OBJECT

    public:
        MainWindow(PlaybackControl *pb);
        ~MainWindow();
        
    private:
        GameSignalAdapter *_gameSignalAdapter;
        RefboxConfigAdapter * refboxConfig;

        std::vector<QAction*> _robotViewActions;
        std::vector<QAction*> _robotHeightmapActions;
        std::vector<WidgetBase *> _widgets;
        PlaybackControl *_pbControl;

    protected:
        bool eventFilter(QObject *obj, QEvent *event); // handle Qt events from parent window
        virtual void resizeEvent(QResizeEvent *event);

    public Q_SLOTS:
        void switchViewToWorld(void);
        void switchViewToGaussianWorld(void);
        void switchViewToBallMeasurements(void);
        void switchViewToObstacleMeasurements(void);
        void switchViewToVision(void);
        void switchViewToPathPlanning(void);
        void switchViewToTeamplay(void);
        void switchViewToTeam(void);
        void switchViewToRobot(int robotId);

    private Q_SLOTS:
        void showSettingsDialog(void);

    };
}

#endif // MAINWINDOW_H
