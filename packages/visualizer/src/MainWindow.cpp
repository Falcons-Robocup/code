 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include <map>

// Internal:
#include "int/MainWindow.h"
#include "int/RtdbGameSignalAdapter.h"
#include "int/RtdbRefboxConfigAdapter.h"
#include "int/ConfigurationManager.h"
#include "int/widgets/Table/TableWidget.h"
#include "int/widgets/Playback/PlaybackWidget.h"
#include "int/widgets/Field/EventHandlers/FieldMouseHoverEventHandler.h"
#include "int/widgets/SettingsDialog.h"

using namespace Visualizer;
using std::map;

static RefboxConfigAdapter::TeamColor teamColorFromString(QString value);
static RefboxConfigAdapter::PlayingField playingFieldFromString(QString value);
static RefboxConfigAdapter::TTAConfiguration ttaConfigurationFromString(QString value);

MainWindow::MainWindow(PlaybackControl *pb)
    : _pbControl(pb)
{
    setupUi(this);
    this->installEventFilter(this); // Set this window as handler of Qt events

    // Configure QSettings
    QCoreApplication::setOrganizationName("Falcons");
    QCoreApplication::setApplicationName("visualizer");

    // StackOverflow.com/a/9638420/1980516
    this->centralWidget()->setAttribute(Qt::WA_TransparentForMouseEvents);
    setMouseTracking(true);
    this->setMouseTracking(true);
    fieldWidget->setMouseTracking(true);

    // Connect window management signals
    connect(actionQuit, SIGNAL(triggered()), this, SLOT(close()));
    connect(actionViewWorldView, SIGNAL(triggered()), this, SLOT(switchViewToWorld()));
    connect(actionViewGaussianWorldView, SIGNAL(triggered()), this, SLOT(switchViewToGaussianWorld()));
    connect(actionViewGaussianMeasurementsView, SIGNAL(triggered()), this, SLOT(switchViewToGaussianMeasurements()));
    connect(actionViewVisionView, SIGNAL(triggered()), this, SLOT(switchViewToVision()));
    connect(actionViewPathPlanningView, SIGNAL(triggered()), this, SLOT(switchViewToPathPlanning()));
    connect(actionViewTeamplayView, SIGNAL(triggered()), this, SLOT(switchViewToTeamplay()));
    connect(actionHeightmapNone, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToNone()));
    connect(actionHeightmapDefendAttackingOpponent, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToDefendAttackingOpponent()));
    connect(actionHeightmapDribble, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToDribble()));
    connect(actionHeightmapMoveToFreeSpot, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToMoveToFreeSpot()));
    connect(actionHeightmapPositionForOppSetpiece, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToPositionForOppSetpiece()));
    connect(actionHeightmapPositionForOwnSetpiece, SIGNAL(triggered()), fieldWidget, SLOT(switchHeightmapToPositionForOwnSetpiece()));
    connect(actionResetZoomPanRotate, SIGNAL(triggered()), fieldWidget, SLOT(resetZoomPanRotate()));
    connect(actionFlip, SIGNAL(toggled(bool)), fieldWidget, SLOT(flip(bool)));

    // Allow enabling/disabling the dock widgets via the 'Window' menu option
    menuWindows->addAction(clockDockWidget->toggleViewAction());
    menuWindows->addAction(tableDockWidget->toggleViewAction());
    menuWindows->addAction(eventLogDockWidget->toggleViewAction());

    // Add actions for Settings-dialogs
    connect(toolButton, SIGNAL(pressed()), this, SLOT(showSettingsDialog()));

    // Add actions to switch to team view
    connect(actionViewTeam, SIGNAL(triggered()), this, SLOT(switchViewToTeam()));
    switchViewToTeam();

    // Add actions to switch to robot view
    {
        QSignalMapper* mapper = new QSignalMapper(); // Use QSignalMapper to supply robot id to the signal
        connect(mapper, SIGNAL(mapped(int)), this, SLOT(switchViewToRobot(int)));
        boost::format format_robot("Robot %1%");
        for (int id = 1; id < _NR_OF_ROBOTS_PER_TEAM + 1; id++)
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
    _gameSignalAdapter = new RtdbGameSignalAdapter();
    for (size_t i = 0; i < _widgets.size(); ++i)
    {
        _widgets[i]->getSignalSubscriber()->subscribe(_gameSignalAdapter);
    }

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

    refboxConfig = new RtdbRefboxConfigAdapter();

    // Restore widget layouts
    for (size_t i = 0; i < _widgets.size(); ++i)
    {
        _widgets[i]->restoreState();
    }

    fieldWidget->resetZoomPanRotate();
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

    delete refboxConfig;
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
        for (size_t i = 0; i < _widgets.size(); ++i)
        {
            _widgets[i]->saveState();
        }

        settings.sync();
    }

    return false;
}

void MainWindow::switchViewToWorld(void)
{
    for (size_t i = 0; i < _widgets.size(); ++i)
    {
        _widgets[i]->getSignalSubscriber()->setSignalMode(WORLD);
    }
    actionViewWorldView->setChecked(true);
}

void MainWindow::switchViewToGaussianWorld(void)
{
    for (size_t i = 0; i < _widgets.size(); ++i)
    {
        _widgets[i]->getSignalSubscriber()->setSignalMode(GAUSSIAN_WORLD);
    }
    actionViewWorldView->setChecked(true);
}

void MainWindow::switchViewToGaussianMeasurements(void)
{
    for (size_t i = 0; i < _widgets.size(); ++i)
    {
        _widgets[i]->getSignalSubscriber()->setSignalMode(GAUSSIAN_MEASUREMENTS);
    }
    actionViewWorldView->setChecked(true);
}

void MainWindow::switchViewToVision(void)
{
    for (size_t i = 0; i < _widgets.size(); ++i)
    {
        _widgets[i]->getSignalSubscriber()->setSignalMode(VISION);
    }
    actionViewVisionView->setChecked(true);
}

void MainWindow::switchViewToPathPlanning(void)
{
    for (size_t i = 0; i < _widgets.size(); ++i)
    {
        _widgets[i]->getSignalSubscriber()->setSignalMode(PATHPLANNING);
    }
    actionViewPathPlanningView->setChecked(true);
}

void MainWindow::switchViewToTeamplay(void)
{
    for (size_t i = 0; i < _widgets.size(); ++i)
    {
        _widgets[i]->getSignalSubscriber()->setSignalMode(TEAMPLAY);
    }
    actionViewTeamplayView->setChecked(true);
}

void MainWindow::switchViewToTeam(void)
{
    for (size_t i = 0; i < _widgets.size(); ++i)
    {
        _widgets[i]->getTeamRobotSelection()->setTeamMode();
    }
    actionViewTeam->setChecked(true);
}

void MainWindow::switchViewToRobot(int robotId)
{
    for (size_t i = 0; i < _widgets.size(); ++i)
    {
        _widgets[i]->getTeamRobotSelection()->setRobotMode((uint8_t)robotId);
    }
    _robotViewActions[robotId - 1]->setChecked(true);
}

void MainWindow::showSettingsDialog()
{
    Settings::SettingsDialog dialog;
    QDialog::DialogCode result = (QDialog::DialogCode)dialog.exec();
    if (result == QDialog::Accepted)
    {
        fieldWidget->reloadSettings();

        // Need to update our settings here somehow...
        QSettings settings;
        QString teamColor = settings.value(Settings::teamColorSetting).toString();
        QString playingField = settings.value(Settings::playingFieldSetting).toString();
        QString ttaConfig = settings.value(Settings::ttaConfigSetting).toString();

        refboxConfig->setTeamColor(teamColorFromString(teamColor));
        refboxConfig->setPlayingField(playingFieldFromString(playingField));
        refboxConfig->setTTAConfiguration(ttaConfigurationFromString(ttaConfig));
    }
}

static RefboxConfigAdapter::TeamColor teamColorFromString(QString value)
{
    map<QString, RefboxConfigAdapter::TeamColor> teamColorMap = {
            {"CYAN", RefboxConfigAdapter::TeamColor::CYAN},
            {"MAGENTA", RefboxConfigAdapter::TeamColor::MAGENTA},
    };

    return teamColorMap[value];
}

static RefboxConfigAdapter::PlayingField playingFieldFromString(QString value)
{
    map<QString, RefboxConfigAdapter::PlayingField> playingFieldMap = {
            {"FIELD_A", RefboxConfigAdapter::PlayingField::FIELD_A},
            {"FIELD_B", RefboxConfigAdapter::PlayingField::FIELD_B},
            {"FIELD_C", RefboxConfigAdapter::PlayingField::FIELD_C},
            {"FIELD_D", RefboxConfigAdapter::PlayingField::FIELD_D},
            {"FIELD_FALCONS", RefboxConfigAdapter::PlayingField::FIELD_FALCONS},
            {"FIELD_LOCALHOST", RefboxConfigAdapter::PlayingField::FIELD_LOCALHOST},
    };

    return playingFieldMap[value];
}

static RefboxConfigAdapter::TTAConfiguration ttaConfigurationFromString(QString value)
{
    std::string s = value.toStdString();
    tprintf("ttaConfigurationFromString %s", s.c_str());
    map<QString, RefboxConfigAdapter::TTAConfiguration> ttaConfigMap = {
            {"NONE", RefboxConfigAdapter::TTAConfiguration::NONE},
            {"FRONT_LEFT", RefboxConfigAdapter::TTAConfiguration::FRONT_LEFT},
            {"FRONT_RIGHT", RefboxConfigAdapter::TTAConfiguration::FRONT_RIGHT},
            {"BACK_LEFT", RefboxConfigAdapter::TTAConfiguration::BACK_LEFT},
            {"BACK_RIGHT", RefboxConfigAdapter::TTAConfiguration::BACK_RIGHT},
    };

    return ttaConfigMap.at(value);
}

