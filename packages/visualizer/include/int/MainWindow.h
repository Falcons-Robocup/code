// Copyright 2015-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <ui_MainWindow.h>

#include <QtGui>

#include <iostream>
#include <string>
#include <memory>

// Internal:
#include "int/adapters/RtdbGameSignalAdapter.h"
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
        std::shared_ptr<cDbConnection>          _db_connection_A = nullptr;
        std::shared_ptr<cDbConnection>          _db_connection_B = nullptr;
        std::unique_ptr<RtdbGameSignalAdapter>  _gameSignalAdapter = nullptr;

        QAction*                    _actionViewTeamB = nullptr;
        std::vector<QAction*>       _robotViewActions;
        std::vector<QAction*>       _robotHeightmapActions;
        std::vector<QAction*>       _roleHeightmapActions;
        
        std::vector<WidgetBase *>   _widgets;
        PlaybackControl*            _pbControl = nullptr;

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
        void switchViewToTeamB(void);
        void switchViewToRobot(int robotId);

    private Q_SLOTS:
        void setDarkTheme(void);
        void setLightTheme(void);
    };
}

#endif // MAINWINDOW_H
