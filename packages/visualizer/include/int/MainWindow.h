 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
        void switchViewToGaussianMeasurements(void);
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
