// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CAMERAVIEWERALL_HPP
#define CAMERAVIEWERALL_HPP

#include <QGraphicsScene>

#include "cameraViewer.hpp"
#include "decoder.hpp" // for objectSt and statsSt

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class mainWindow; // forward declaration

class cameraViewerAll {
public:
    cameraViewerAll(Ui::MainWindow *ui);
    void draw(int camera, int robot, QVector<objectSt> objects, statSt statistics);

private:
    Ui::MainWindow *ui = nullptr;

#define CAM_WINDOWS 6
    QGraphicsScene *camSceneFront[CAM_WINDOWS];
    QGraphicsScene *camSceneLeft[CAM_WINDOWS];
    QGraphicsScene *camSceneRight[CAM_WINDOWS];

    cameraViewer *camViewFront[CAM_WINDOWS];
    cameraViewer *camViewLeft[CAM_WINDOWS];
    cameraViewer *camViewRight[CAM_WINDOWS];
};

#endif // CAMERAVIEWERALL_HPP
