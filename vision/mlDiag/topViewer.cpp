// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QDebug>
#include <QGraphicsScene>
#include <QtMath>
#include <QVector>

#include "mainWindow.hpp"
#include "ui_mainWindow.h"
#include "topViewer.hpp"

topViewer::topViewer(Ui::MainWindow *ui) {
    this->ui = ui;
    topScene = new QGraphicsScene();

    this->ui->topView->setScene(topScene);
    this->ui->topView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    this->ui->topView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    topScene->setBackgroundBrush(QBrush(Qt::darkGreen));

    penDotLine.setColor(Qt::lightGray);
    penDotLine.setStyle(Qt::DotLine);

    rfloor = new robotFloor(topScene);
}

void topViewer::pre() {
    float topWidth = ui->topView->width();
    float topHeight = ui->topView->height();

    rfloor->update(topWidth, topHeight); // also clears the scene

    for( int robotM1 = 0; robotM1 < ROBOTS; robotM1++ ) {
        robotIsShown[robotM1] = false;
    }
}

void topViewer::post(quint64 captureLatest) {
    rfloor->addTopViewText(captureLatest);
}

void topViewer::drawRobot(int robot, float xOffset, float yOffset) {
    float xScale = rfloor->getXScale();
    float yScale = rfloor->getYScale();

    // draw robot in center of field
    rfloor->addEllipse(xOffset, yOffset, xScale * 0.5, yScale * 0.5, QPen(Qt::red), QBrush(Qt::transparent) );
    rfloor->addLine(xOffset, yOffset - yScale * 0.25, xOffset, yOffset - 2 * yScale * 0.25, QPen(Qt::red));

    rfloor->addText(xOffset + 0.01, yOffset + 0.01, QString::asprintf("robot %1d", robot), 12);
}

void topViewer::drawCamera(int camera, QVector<objectSt> objects, int robot, float xOffset, float yOffset) {
    float xScale = rfloor->getXScale();
    float yScale = rfloor->getYScale();

    // draw the robot only once for all camera's
    if( ! robotIsShown[robot - 1] ) {
        drawRobot(robot, xOffset, yOffset);
        robotIsShown[robot - 1] = true;
    }

    for( int ii = 0; ii < objects.size(); ii++ ) {
        // azimuth definition: 0 degrees is north and 90 degrees is east
        float azimuth = objects[ii].azimuth + camera * M_PI/2.0; // camera's are placed with 90 degrees offset

        // enable next line to show all angles from robot to objects
        // rfloor->addLineAngle(xOffset, yOffset, azimuth, 0.3, 2, QPen(Qt::red));

        float xCenter = xOffset + xScale * objects[ii].radius * qSin(azimuth);
        float yCenter = yOffset - yScale * objects[ii].radius * qCos(azimuth);

        // determine if x or y is first out of range
        // the following combinations are possible
        // x >= 0.0 && x <= 1.0 && y >= 0.0 && y <= 1.0 : all in range
        // x >= 0.0 && x <= 1.0 && y < 0.0 : y out of range
        // x >= 0.0 && x <= 1.0 && y > 1.0 : y out of range
        // x < 0.0 && y >= 0.0 && y <= 1.0 : x out of range
        // x > 1.0 && y >= 0.0 && y <= 1.0 : x out of range
        // x > 1.0 && y > 1.0 : both out of range
        // x > 1.0 && y < 0.0 : both out of range
        // x < 0.0 && y > 1.0 : both out of range
        // x < 0.0 && y < 0.0 : both out of range

        bool insideTopScene = false;

        if( xCenter >= 0.0 && xCenter <= 1.0 && yCenter >= 0.0 && yCenter <= 1.0 ) {
            // all in range, do nothing
            insideTopScene = true;
        } else if( xCenter >= 0.0 && xCenter <= 1.0 && yCenter < 0.0 ) { // yCenter out of range
            xCenter = xOffset + (xScale/yScale) * yOffset * qTan(azimuth);
            yCenter = 0.0;
            // qDebug() <<"yCenter < 0.0 xCenter" << xCenter;
        } else if( xCenter >= 0.0 && xCenter <= 1.0 && yCenter > 1.0 ) { // yCenter out of range
            xCenter = xOffset - (xScale/yScale) * (1.0 - yOffset) * qTan(azimuth);
            yCenter = 1.0;
            // qDebug() <<"yCenter > 1.0 xCenter" << xCenter;
        } else if( xCenter < 0.0 && yCenter >= 0.0 && yCenter <= 1.0 ) { // xCenter out of range
            xCenter = 0.0;
            yCenter = yOffset + (yScale/xScale) *  xOffset / qTan(azimuth);
            // qDebug() <<"xCenter < 0.0 yCenter" << yCenter;
        } else if( xCenter > 1.0 && yCenter >= 0.0 && yCenter <= 1.0 ) { // xCenter out of range
            xCenter = 1.0;
            yCenter = yOffset - (yScale/xScale) * (1.0 - xOffset) / qTan(azimuth);
            // qDebug() <<"xCenter > 1.0 yCenter" << yCenter;
        } else if( xCenter > 1.0 && yCenter > 1.0 ) { // both out of range
            float xCenterTest = xOffset - (xScale/yScale) * (1.0 - yOffset) * qTan(azimuth);
            float yCenterTest = yOffset - (yScale/xScale) * (1.0 - xOffset) / qTan(azimuth);
            if( yCenterTest <= 1.0 ) { // xCenter first out of range
                xCenter = 1.0;
                yCenter = yCenterTest;
                // qDebug() <<"xCenter > 1.0 && yCenter > 1.0 yCenter" << yCenter;
            } else if( xCenterTest <= 1.0 ) { // yCenter first out of range
                xCenter = xCenterTest;
                yCenter = 1.0;
                // qDebug() <<"xCenter > 1.0 && yCenter > 1.0 xCenter" << xCenter;
            } else {
                qDebug() << "ERROR   xCenter > 1.0 && yCenter > 1.0 cannot determine border, xCenter" << xCenterTest << "yCenter" << yCenterTest;
            }
        } else if( xCenter > 1.0 && yCenter < 0.0 ) { // both out of range
            float xCenterTest = xOffset + (xScale/yScale) * yOffset * qTan(azimuth);
            float yCenterTest = yOffset - (yScale/xScale) * (1.0 - xOffset) / qTan(azimuth);
            if( yCenterTest >= 0.0 ) { // xCenter first out of range
                xCenter = 1.0;
                yCenter = yCenterTest;
                // qDebug() <<"xCenter > 1.0 && yCenter < 0.0 yCenter" << yCenter;
            } else if( xCenterTest <= 1.0 ) { // yCenter first out of range
                xCenter = xCenterTest;
                yCenter = 0.0;
                // qDebug() <<"xCenter > 1.0 && yCenter < 0.0 xCenter" << xCenter;
            } else {
                qDebug() << "ERROR   xCenter > 1.0 && yCenter < 0.0 cannot determine border, xCenter" << xCenterTest << "yCenter" << yCenterTest;
            }
        } else if( xCenter < 0.0 && yCenter < 0.0 ) { // both out of range
            float xCenterTest = xOffset + (xScale/yScale) * yOffset * qTan(azimuth);
            float yCenterTest = yOffset + (yScale/xScale) *  xOffset / qTan(azimuth);
            if( yCenterTest >= 0.0 ) { // xCenter first out of range
                xCenter = 0.0;
                yCenter = yCenterTest;
                // qDebug() <<"xCenter < 0.0 && yCenter < 0.0 yCenter" << yCenter;
            } else if( xCenterTest >= 0.0 ) { // yCenter first out of range
                xCenter = xCenterTest;
                yCenter = 0.0;
                // qDebug() <<"xCenter < 0.0 && yCenter < 0.0 xCenter" << xCenter;
            } else {
                qDebug() << "ERROR   xCenter < 0.0 && yCenter < 0.0 cannot determine border, xCenter" << xCenterTest << "yCenter" << yCenterTest;
            }
        } else if( xCenter < 0.0 && yCenter > 1.0 ) { // both out of range
            float xCenterTest = xOffset - (xScale/yScale) * (1.0 - yOffset) * qTan(azimuth);
            float yCenterTest = yOffset + (yScale/xScale) *  xOffset / qTan(azimuth);
            if( yCenterTest <= 1.0 ) { // xCenter first out of range
                xCenter = 0.0;
                yCenter = yCenterTest;
                // qDebug() <<"xCenter < 0.0 && yCenter > 1.0 yCenter" << yCenter;
            } else if( xCenterTest >= 0.0 ) { // yCenter first out of range
                xCenter = xCenterTest;
                yCenter = 1.0;
                // qDebug() <<"xCenter < 0.0 && yCenter > 1.0 xCenter" << xCenter;
            } else {
                qDebug() << "ERROR   xCenter < 0.0 && yCenter > 1.0 cannot determine border, xCenter" << xCenterTest << "yCenter" << yCenterTest;
            }
        } else {
            qDebug() << "ERROR   invalid xCenter" << xCenter << "yCenter" << yCenter << "combination";
        }

        if( xCenter < 0.0 || xCenter > 1.0 || yCenter < 0.0 || yCenter > 1.0 ) {
            qDebug() << "ERROR   xCenter" << xCenter << "or yCenter" << yCenter << "out of range";
        }

        // draw the object
        QBrush brush = objects[ii].color;
        if (objects[ii].elevation > 0.0 ) {
            brush = Qt::transparent;
        }
        if( insideTopScene ) {
            // position inside top view, display as circle
            float width = xScale * 0.4;
            float height = yScale * 0.4;
            rfloor->addEllipse(xCenter, yCenter, width, height, QPen(objects[ii].color), brush );
        } else {
            // position outside top view, display as cross
            QPen crossPen(objects[ii].color);
            crossPen.setWidth(3);
            float width = xScale * 0.2;
            float height = yScale * 0.2;
            rfloor->addLine(xCenter-width, yCenter-height, xCenter+width, yCenter+height, crossPen);
            rfloor->addLine(xCenter-width, yCenter+height, xCenter+width, yCenter-height, crossPen);
        }

        // draw the lines from the robot to the objects, but do not draw them inside the robot and inside the object
        float xRobot = xOffset + xScale * 0.25 * qSin(azimuth);
        float yRobot = yOffset - yScale * 0.25 * qCos(azimuth);
        float xObject = xCenter - xScale * 0.2 * qSin(azimuth);
        float yObject = yCenter + yScale * 0.2 * qCos(azimuth);
        rfloor->addLine(xRobot, yRobot, xObject, yObject, penDotLine);

        // show confidence for the object
        rfloor->addText(xCenter + 0.006, yCenter + 0.006, QString::asprintf("%.0f%%", objects[ii].confidence*100.0));
    }
}
