 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include <QApplication>
#include <GL/glut.h>
#include <signal.h>
#include <string>

#include "tracing.hpp"

// Internal:
#include "int/MainWindow.h"


using namespace std;

QApplication* application = NULL;

int main(int argc, char *argv[])
{
    string nodename = "visualizer";

    INIT_TRACE;

    application = new QApplication(argc, argv);
    glutInit(&argc,argv);

    // Playback control
    bool fileMode = (argc > 1);
    std::string logFileName = "";
    if (fileMode)
    {
        logFileName = argv[1];
    }
    printf("initializing PlaybackControl\n"); fflush(stdout);
    TRACE("initializing PlaybackControl");
    PlaybackControl *pbControl = new PlaybackControl(fileMode, logFileName);

    // Setting up the visualizer main window
    printf("initializing Visualizer::MainWindow\n"); fflush(stdout);
    TRACE("initializing Visualizer::MainWindow");
    Visualizer::MainWindow * visualizerWindow = new Visualizer::MainWindow(pbControl);
    visualizerWindow->show();

    // Setting up fusion
    printf("setStyle fusion\n"); fflush(stdout);
    TRACE("setStyle fusion");
    application->setStyle("fusion");

    // Starting gui loop
    TRACE("start gui loop");
    int ret = application->exec();

    // Shutting down
    if (visualizerWindow != NULL) 
    {
        delete visualizerWindow; 
        visualizerWindow = NULL;
    }
    
    return ret;
}
