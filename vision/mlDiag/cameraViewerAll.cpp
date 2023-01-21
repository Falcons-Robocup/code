// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QDebug>
#include <QGraphicsScene>
#include <QVector>

#include "cameraViewerAll.hpp"
#include "mainWindow.hpp"
#include "ui_mainWindow.h"

cameraViewerAll::cameraViewerAll(Ui::MainWindow *ui) {
    this->ui = ui;

    if( CAM_WINDOWS != 6 ) {
        qDebug() << "ERROR   only design for 6 camera windows, but configured for" << CAM_WINDOWS;
        exit(EXIT_FAILURE);
    }

    for( size_t ii = 0; ii < CAM_WINDOWS; ii++ ) {
        camSceneFront[ii] = new QGraphicsScene();
        camSceneLeft[ii] = new QGraphicsScene();
        camSceneRight[ii] = new QGraphicsScene();
    }

    this->ui->camView0Front->setScene(camSceneFront[0]);
    this->ui->camView0Front->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView0Front->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView0Left->setScene(camSceneLeft[0]);
    this->ui->camView0Left->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView0Left->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView0Right->setScene(camSceneRight[0]);
    this->ui->camView0Right->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView0Right->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    this->ui->camView1Front->setScene(camSceneFront[1]);
    this->ui->camView1Front->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView1Front->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView1Left->setScene(camSceneLeft[1]);
    this->ui->camView1Left->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView1Left->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView1Right->setScene(camSceneRight[1]);
    this->ui->camView1Right->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView1Right->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    this->ui->camView2Front->setScene(camSceneFront[2]);
    this->ui->camView2Front->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView2Front->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView2Left->setScene(camSceneLeft[2]);
    this->ui->camView2Left->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView2Left->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView2Right->setScene(camSceneRight[2]);
    this->ui->camView2Right->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView2Right->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    this->ui->camView3Front->setScene(camSceneFront[3]);
    this->ui->camView3Front->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView3Front->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView3Left->setScene(camSceneLeft[3]);
    this->ui->camView3Left->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView3Left->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView3Right->setScene(camSceneRight[3]);
    this->ui->camView3Right->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView3Right->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    this->ui->camView4Front->setScene(camSceneFront[4]);
    this->ui->camView4Front->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView4Front->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView4Left->setScene(camSceneLeft[4]);
    this->ui->camView4Left->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView4Left->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView4Right->setScene(camSceneRight[4]);
    this->ui->camView4Right->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView4Right->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    this->ui->camView5Front->setScene(camSceneFront[5]);
    this->ui->camView5Front->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView5Front->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView5Left->setScene(camSceneLeft[5]);
    this->ui->camView5Left->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView5Left->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView5Right->setScene(camSceneRight[5]);
    this->ui->camView5Right->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->camView5Right->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    // TODO: dynamic robot assignment to free windows (in order)
    this->ui->camGroup0->setTitle("robot 1");
    this->ui->camGroup1->setTitle("robot 2");
    this->ui->camGroup2->setTitle("robot 3");
    this->ui->camGroup3->setTitle("robot 4");
    this->ui->camGroup4->setTitle("robot 5");
    this->ui->camGroup5->setTitle("robot 6");


    for( size_t ii = 0; ii < CAM_WINDOWS; ii++ ) {
        camViewFront[ii] = new cameraViewer(camSceneFront[ii]);
        camViewLeft[ii] = new cameraViewer(camSceneLeft[ii]);
        camViewRight[ii] = new cameraViewer(camSceneRight[ii]);
    }
}

void cameraViewerAll::draw(int robot, int camera, QVector<objectSt> objects, statSt statistics) {
    if( robot < 1 || robot > CAM_WINDOWS ) {
        qDebug() << "ERROR   robot index" << robot << "out for range";
        exit(EXIT_FAILURE);
    }

    // scale the size of the camera windows
    // the size is the saem for all camera windows
    uint width = ui->camView0Left->width();
    uint height = ui->camView0Left->height();

    // TODO: dynamic robot assignment to free windows (in order)
    if( camera == 0 ) { // front
        camViewFront[robot-1]->drawCamScene(width, height, objects, statistics);
    } else if( camera == 1 ) { // right
        camViewRight[robot-1]->drawCamScene(width, height, objects, statistics);
    } else if( camera == 3 ) { // left
        camViewLeft[robot-1]->drawCamScene(width, height, objects, statistics);
    } // NOTE: back camera (index 2) is not displayed
}
