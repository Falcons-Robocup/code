 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include <QApplication>
#include <GL/glut.h>
#include <signal.h>
#include <string>
#include <ros/ros.h>

// Internal:
#include "int/MainWindow.h"

using namespace std;

QApplication* application = NULL;

void closeApplication(int sig)
{
    if (sig == SIGINT)
    {
        if (application != NULL)
        {
            ros::shutdown();
            application->closeAllWindows();
        }
    }
}

int main(int argc, char *argv[])
{
    string nodename = "visualizer";
    ros::init(argc, argv, nodename);

    application = new QApplication(argc, argv);
    
    glutInit(&argc,argv);

    // Setting up the visualizer main window
    Visualizer::MainWindow * visualizerWindow = new Visualizer::MainWindow();
    visualizerWindow->show();

    // Setting up QPlastiqueStyle
    QApplication::setStyle(new QPlastiqueStyle);

    // Override default ROS SIGINT handler
    // Must be done after first NodeHandle is created.
    if (signal(SIGINT, closeApplication) == SIG_ERR)
    {
        cerr << nodename + " :: Register SIGINT Error" << endl;
        return 1;
    }

    // Starting gui loop
    int ret = application->exec();

    // Shutting down
    if (visualizerWindow != NULL) 
    {
        delete visualizerWindow; 
        visualizerWindow = NULL;
    }
    
    return ret;
}
