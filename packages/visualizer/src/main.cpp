// Copyright 2015-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
