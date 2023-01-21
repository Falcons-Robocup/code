// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPVIEWER_HPP
#define TOPVIEWER_HPP

#include <QGraphicsScene>
#include <QPen>

#include "decoder.hpp" // for objectSt and statsSt
#include "robotFloor.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class mainWindow; // forward declaration

class topViewer {
public:
    topViewer(Ui::MainWindow *ui);
    void drawCamera(int camera, QVector<objectSt> objects, int robot, float xOffset, float yOffset);
    void drawRobot(int robot, float xOffset, float yOffset);
    void post(quint64 captureLates);
    void pre();

private:
    robotFloor *rfloor = nullptr;
    QGraphicsScene *topScene = nullptr;
    Ui::MainWindow *ui = nullptr;

    bool robotIsShown[ROBOTS];
    QPen penDotLine;



};

#endif // TOPVIEWER_HPP
