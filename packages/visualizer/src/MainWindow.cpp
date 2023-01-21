// Copyright 2015-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <map>

// Internal:
#include "int/MainWindow.h"
#include "int/adapters/RtdbGameSignalAdapter.h"
#include "int/ConfigurationManager.h"
#include "int/widgets/Table/TableWidget.h"
#include "int/widgets/Playback/PlaybackWidget.h"
#include "int/widgets/Field/EventHandlers/FieldMouseHoverEventHandler.h"

#include "falconsCommon.hpp" // isSimulatedEnvironment

using namespace Visualizer;
using std::map;

const std::string RTDB_TEAM_A = std::string(RTDB_STORAGE_TEAMS)+'A';
const std::string RTDB_TEAM_B = std::string(RTDB_STORAGE_TEAMS)+'B';

MainWindow::MainWindow(PlaybackControl *pb)
    : _pbControl(pb)
{
    setupUi(this);
    this->installEventFilter(this); // Set this window as handler of Qt events

    // Configure QSettings
    QCoreApplication::setOrganizationName("Falcons");
    QCoreApplication::setApplicationName("Visualizer");

    // // StackOverflow.com/a/9638420/1980516
    // this->centralWidget()->setAttribute(Qt::WA_TransparentForMouseEvents);
    // setMouseTracking(true);
    // this->setMouseTracking(true);
    // fieldWidget->setMouseTracking(true);

    // Connect window management signals
    connect(actionViewWorldView, SIGNAL(triggered()), this, SLOT(switchViewToWorld()));
    connect(actionViewGaussianWorldView, SIGNAL(triggered()), this, SLOT(switchViewToGaussianWorld()));
    connect(actionViewBallMeasurementsView, SIGNAL(triggered()), this, SLOT(switchViewToBallMeasurements()));
    connect(actionViewObstacleMeasurementsView, SIGNAL(triggered()), this, SLOT(switchViewToObstacleMeasurements()));
    connect(actionViewVisionView, SIGNAL(triggered()), this, SLOT(switchViewToVision()));
    connect(actionViewPathPlanningView, SIGNAL(triggered()), this, SLOT(switchViewToPathPlanning()));
    connect(actionViewTeamplayView, SIGNAL(triggered()), this, SLOT(switchViewToTeamplay()));
    connect(actionHeightmapNone, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToNone()));
    connect(actionActiveHeightmap, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToActiveHeightmap()));
    connect(actionHeightmapDefendAttackingOpponent, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToDefendAttackingOpponent()));
    connect(actionHeightmapDribble, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToDribble()));
    connect(actionHeightmapMoveToFreeSpot, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToMoveToFreeSpot()));
    connect(actionHeightmapPositionForOppSetpiece, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToPositionForOppSetpiece()));
    connect(actionHeightmapPositionAttackerForOwnSetpiece, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToPositionAttackerForOwnSetpiece()));
    connect(actionResetZoomPanRotate, SIGNAL(triggered()), fieldWidget, SLOT(resetZoomPanRotate()));
    connect(actionFlip, SIGNAL(toggled(bool)), fieldWidget, SLOT(flip(bool)));

    connect(actionDark, SIGNAL(triggered()), this, SLOT(setDarkTheme()));
    connect(actionLight, SIGNAL(triggered()), this, SLOT(setLightTheme()));

    // Allow enabling/disabling the dock widgets via the 'Window' menu option
    menuWindows->addAction(clockDockWidget->toggleViewAction());
    menuWindows->addAction(tableDockWidget->toggleViewAction());
    menuWindows->addAction(eventLogDockWidget->toggleViewAction());
    menuWindows->addAction(matchStateDockWidget->toggleViewAction());
    menuWindows->addAction(playbackDockWidget->toggleViewAction());
    menuWindows->addAction(robotStatusDockWidget->toggleViewAction());

    // Add actions to switch to team view
    connect(actionViewTeam, SIGNAL(triggered()), this, SLOT(switchViewToTeam()));

    if (isSimulatedEnvironment())
    {
        std::string message("Team B");
        _actionViewTeamB = new QAction(actionViewTeamGroup);

        connect(_actionViewTeamB, SIGNAL(triggered()), this, SLOT(switchViewToTeamB()));

        _actionViewTeamB->setCheckable(true);
        _actionViewTeamB->setText(QApplication::translate("MainWindow", message.c_str(), 0));
        menuViewTeamRobot->addAction(_actionViewTeamB);

        _db_connection_B = std::make_shared<cDbConnection>(RTDB_TEAM_B);
    }

    // Add actions to switch to robot view
    {
        QSignalMapper* mapper = new QSignalMapper(); // Use QSignalMapper to supply robot id to the signal
        connect(mapper, SIGNAL(mapped(int)), this, SLOT(switchViewToRobot(int)));
        boost::format format_robot("Robot %1%");
        for (int id = 1; id <= _NR_OF_ROBOTS_PER_TEAM; id++)
        {
            std::string message_robot = boost::str(format_robot % id);
            QAction* robotAction = new QAction(actionViewTeamGroup);

            mapper->setMapping(robotAction, id);
            connect(robotAction, SIGNAL(triggered()), mapper, SLOT(map()));

            robotAction->setCheckable(true);
            robotAction->setText(QApplication::translate("MainWindow", message_robot.c_str(), 0));
            menuViewTeamRobot->addAction(robotAction);
            _robotViewActions.push_back(robotAction);
        }
    }

    // Add actions to switch heightmap for robot
    {
        QSignalMapper* mapper = new QSignalMapper(); // Use QSignalMapper to supply robot id to the signal
        connect(mapper, SIGNAL(mapped(int)), fieldWidget, SLOT(switchHeightmapForRobot(int)));
        boost::format format_robot("Robot %1%");
        for (int id = 1; id < _NR_OF_ROBOTS_PER_TEAM + 1; id++)
        {
            std::string message_robot = boost::str(format_robot % id);
            QAction* robotAction = new QAction(actionHeightmapRobotGroup);

            mapper->setMapping(robotAction, id);
            connect(robotAction, SIGNAL(triggered()), mapper, SLOT(map()));

            robotAction->setCheckable(true);
            robotAction->setText(QApplication::translate("MainWindow", message_robot.c_str(), 0));
            menuHeightmapRobot->addAction(robotAction);
            _robotHeightmapActions.push_back(robotAction);
        }
    }

    // Add actions to switch heightmap for role
    {
        std::array<QString, 6> roles{"STOP", "GOALKEEPER", "ATTACKER_MAIN", "ATTACKER_ASSIST", "DEFENDER_MAIN", "DEFENDER_ASSIST"};

        QSignalMapper* mapper = new QSignalMapper(); // Use QSignalMapper to supply role to the signal
        connect(mapper, SIGNAL(mapped(QString)), fieldWidget, SLOT(switchHeightmapForRole(QString)));
        for (const auto& role : roles)
        {
            QAction* roleAction = new QAction(actionHeightmapRobotGroup);
            mapper->setMapping(roleAction, role);
            connect(roleAction, SIGNAL(triggered()), mapper, SLOT(map()));

            roleAction->setCheckable(true);
            roleAction->setText(QApplication::translate("MainWindow", role.toStdString().c_str(), 0));
            menuHeightmapRobot->addAction(roleAction);
            _roleHeightmapActions.push_back(roleAction);
        }
    }

    // Initialize table widget nr of columns
    tableWidget->initialize(_NR_OF_ROBOTS_PER_TEAM);

    // Add event handlers to the fieldWidget
    fieldWidget->addFieldEventHandler<FieldMouseHoverEventHandler>();

    // Add all widgets
    _widgets.push_back(fieldWidget);
    _widgets.push_back(eventLogger);
    _widgets.push_back(tableWidget);
    _widgets.push_back(gameTimeClock);
    _widgets.push_back(matchState);
    _widgets.push_back(batteryStatus);
    _widgets.push_back(playbackWidget);
    
    // Register playbackControl in playbackWidget
    playbackWidget->registerPlaybackControl(_pbControl);

    // Connect the signal adapter to each signal subscriber
    _db_connection_A = std::make_shared<cDbConnection>(RTDB_TEAM_A);
    _gameSignalAdapter = std::make_unique<RtdbGameSignalAdapter>(_db_connection_A);
    for (auto w : _widgets)
    {
        w->getSignalSubscriber()->subscribe(_gameSignalAdapter.get());
    }

    switchViewToTeam(); // Wholte team is observed initially.
    switchViewToWorld(); // Initially world view is selected.
    actionHeightmapNone->setChecked(true); // Initially no heightmap is selected.
    _robotHeightmapActions.at(0)->setChecked(true); // Initially heightmap for robot 1 is selected.

    // Restore previous window state
    QSettings settings;
    if (settings.contains("mainwindow") && settings.value("mainwindow").toInt() == 1)
    {
        restoreGeometry(settings.value("mainwindow/geometry").toByteArray());
        restoreState(settings.value("mainwindow/windowState").toByteArray());
    }

    // Restore widget layouts
    for (auto w : _widgets)
    {
        w->restoreState();
    }

    fieldWidget->resetZoomPanRotate();

    showMaximized();
}

MainWindow::~MainWindow()
{
    for (QAction* action : _robotViewActions)
    {
        delete action;
    }
    _robotViewActions.clear();

    for (QAction* action : _robotHeightmapActions)
    {
        delete action;
    }
    _robotHeightmapActions.clear();

    for (QAction* action : _roleHeightmapActions)
    {
        delete action;
    }
    _roleHeightmapActions.clear();    
}

void MainWindow::resizeEvent(QResizeEvent *event) 
{
    QMainWindow::resizeEvent(event);
}

bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

        if (keyEvent->key() == Qt::Key_F1)
        {
            // skip
        }
    }

    if (event->type() == QEvent::Resize)
    {
        // TODO: Move this to a reset-button or something, similar to field widget. This clashes with the saveState functionality of the header.
/*
        int width = tableWidget->width() - tableWidget->style()->pixelMetric(QStyle::PM_ScrollBarExtent) - tableWidget->frameWidth() * 2;
        int columnCount = tableWidget->columnCount();
        float columnWidth = width / columnCount;

        for (int i = 1; i < columnCount; ++i)
        {
            tableWidget->setColumnWidth(i, columnWidth);
        }
*/
    }

    if (event->type() == QEvent::Close)
    {
        // Save window state
        QSettings settings;
        settings.setValue("mainWindow", 1); // QSettings versioning, increase if you rename mainwindow keys.
        settings.setValue("mainWindow/geometry", saveGeometry());
        settings.setValue("mainWindow/windowState", saveState());

        // Save widget layouts
        for (auto w : _widgets)
        {
            w->saveState();
        }

        settings.sync();
    }

    return false;
}

void MainWindow::setDarkTheme(void)
{
    QFile file(":/dark/stylesheet.qss");
    file.open(QFile::ReadOnly | QFile::Text);
    QTextStream stream(&file);
    qApp->setStyleSheet(stream.readAll());
}

void MainWindow::setLightTheme(void)
{
    QFile file(":/light/stylesheet.qss");
    file.open(QFile::ReadOnly | QFile::Text);
    QTextStream stream(&file);
    qApp->setStyleSheet(stream.readAll());
}

void MainWindow::switchViewToWorld(void)
{
    for (auto w : _widgets)
    {
        w->getSignalSubscriber()->setSignalMode(WORLD);
    }
    actionViewWorldView->setChecked(true);
}

void MainWindow::switchViewToGaussianWorld(void)
{
    for (auto w : _widgets)
    {
        w->getSignalSubscriber()->setSignalMode(GAUSSIAN_WORLD);
    }
    actionViewWorldView->setChecked(true);
}

void MainWindow::switchViewToBallMeasurements(void)
{
    for (auto w : _widgets)
    {
        w->getSignalSubscriber()->setSignalMode(BALL_MEASUREMENTS);
    }
    actionViewWorldView->setChecked(true);
}

void MainWindow::switchViewToObstacleMeasurements(void)
{
    for (auto w : _widgets)
    {
        w->getSignalSubscriber()->setSignalMode(OBSTABLE_MEASUREMENTS);
    }
    actionViewWorldView->setChecked(true);
}

void MainWindow::switchViewToVision(void)
{
    for (auto w : _widgets)
    {
        w->getSignalSubscriber()->setSignalMode(VISION);
    }
    actionViewVisionView->setChecked(true);
}

void MainWindow::switchViewToPathPlanning(void)
{
    for (auto w : _widgets)
    {
        w->getSignalSubscriber()->setSignalMode(PATHPLANNING);
    }
    actionViewPathPlanningView->setChecked(true);
}

void MainWindow::switchViewToTeamplay(void)
{
    for (auto w : _widgets)
    {
        w->getSignalSubscriber()->setSignalMode(TEAMPLAY);
    }
    actionViewTeamplayView->setChecked(true);
}

void MainWindow::switchViewToTeam(void)
{
    _gameSignalAdapter->set_db_connection(_db_connection_A);
    for (auto w : _widgets)
    {
        w->getTeamRobotSelection()->setTeamMode();
    }
    actionViewTeam->setChecked(true);
}

void MainWindow::switchViewToTeamB(void)
{
    _gameSignalAdapter->set_db_connection(_db_connection_B);
    for (auto w : _widgets)
    {
        w->getTeamRobotSelection()->setTeamMode();
    }
    _actionViewTeamB->setChecked(true);
}

void MainWindow::switchViewToRobot(int robotId)
{
    for (auto w : _widgets)
    {
        w->getTeamRobotSelection()->setRobotMode((uint8_t)robotId);
    }
    _robotViewActions[robotId - 1]->setChecked(true);
}
