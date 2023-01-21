// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QDebug>
#include <QFont>
#include <QGraphicsTextItem>
#include <QGraphicsView>

#include "cameraViewer.hpp"

#define WIDTH 608 // max 608 (1216)
// #define HEIGHT 800 // max 968 (1936)
#define HEIGHT 864 // max 968 (1936)

cameraViewer::cameraViewer(QGraphicsScene *camScene) {
    this->camScene = camScene;
    camScene->setBackgroundBrush(QBrush(Qt::darkGreen));

    penGrid.setColor(Qt::lightGray);
    penGrid.setStyle(Qt::DotLine);

}

void cameraViewer::drawCamScene(const uint width, const uint height, QVector<objectSt> objects, statSt stats) {
    camWidth = width;
    camHeight = height;

    camScene->clear();

    // scene->setSceneRect(0,0,600,450);
    // scale scene to cam scene area
    // qDebug() << "cam scene width" << width << "height" << height;
    camScene->setSceneRect(0,0,width, height);

    QFont myFont;
    myFont.setPixelSize(10);

    // add cam view to camera viewer window
//    QGraphicsTextItem *text = camScene->addText("cam view");
//    text->setDefaultTextColor(Qt::white);
//    text->setFont(myFont);
//    text->setPos(4,4);

    // show frame counter in the camera viewer
    QGraphicsTextItem *text2 = camScene->addText(QString().asprintf("%7u ", stats.frameId));
    text2->setDefaultTextColor(Qt::white);
    text2->setFont(myFont);
    text2->setPos(0,0);

    // show date time in the camera viewer
//    QDateTime datetime;
//    datetime.setMSecsSinceEpoch(stats.mSecs);
//    QString timeString = datetime.toString("HH:mm:ss.zzz");
//    // qDebug().noquote() << "timeString" << timeString;
//    timeString.chop(1); // change from 1/1000 sec to 1/100 sec
//    QGraphicsTextItem *text3 = camScene->addText(timeString);
//    text3->setDefaultTextColor(Qt::white);
//    text3->setFont(myFont);
//    text3->setPos(4,36);

    // add grid to camera scene
    camScene->addLine(0, camHeight/2.0, camWidth, camHeight/2.0, penGrid);
    camScene->addLine(camWidth/2.0, 0, camWidth/2.0, camHeight, penGrid);
    for( int ii = 1; ii < 5; ii++ ) {
        const qreal thirtyDegX = camWidth * 92.0/WIDTH;
        const qreal thirtyDegY = camHeight * 92.0/HEIGHT;
        camScene->addEllipse(camWidth/2.0-ii*thirtyDegX, camHeight/2.0-ii*thirtyDegY, ii*2.0*thirtyDegX, ii*2.0*thirtyDegY, penGrid);
    }

    for (int ii = 0; ii < objects.size(); ii++ ) {
        drawCamObject(objects[ii]);
    }
}


void cameraViewer::drawCamObject(const objectSt obj) {
    QPen pen(obj.color);
    pen.setWidth(1);

    addRectCamRelCenter(obj.xCenter, obj.yCenter, obj.width, obj.height, pen, QBrush(Qt::transparent));

//    QString description;
//    description.append(QStringLiteral(
//                           "%1\n"
//                           "c %2\n"
//                           "x %3\n"
//                           "y %4"
//                           )              .arg(obj.name)
//                       .arg(obj.confidence,4,'f',2)
//                       .arg(obj.xCenter,5,'f',3)
//                       .arg(obj.yCenter,5,'f',3));

//    QGraphicsTextItem *text = camScene->addText(description);
//    QFont myFont;
//    myFont.setPixelSize(10);
//    text->setDefaultTextColor(Qt::white);
//    text->setFont(myFont);

//    float xText = ( obj.xCenter + obj.width/2.0) * camWidth;
//    if ( xText > ( camWidth - 50 ) ) {
//        xText = camWidth - 50;
//    }

//    float yText = ( obj.yCenter - obj.height/2.0) * camHeight - 5;
//    if ( yText > ( camHeight - 40 ) ) {
//        yText = camHeight - 40;
//    }

//    text->setPos(xText, yText);
}

void cameraViewer::addLineCamRel(const float x0, const float y0, const float x1, const float y1, const QPen pen) {
    camScene->addLine(x0 * camWidth, y0 * camHeight, x1 * camWidth, y1 * camHeight, pen);
}

void cameraViewer::addEllipseCamRelCenter(const float xCenter, const float yCenter, const float radius, const QPen pen) {
    float xLeftPixel = ( xCenter - radius ) * camWidth;
    float yTopPixel = ( yCenter - radius ) * camHeight;
    float widthPixel = 2 * radius * camWidth;
    float heightPixel = 2 * radius * camHeight;

    camScene->addEllipse(xLeftPixel, yTopPixel, widthPixel, heightPixel, pen);
}

void cameraViewer::addRectCamRelCenter(const float xCenter, const float yCenter, const float width, const float height, const QPen pen, const QBrush brush) {
    float xLeftPixel = ( xCenter - width / 2.0 ) * camWidth;
    float yTopPixel = ( yCenter - height / 2.0 ) * camHeight;
    float widthPixel = width * camWidth;
    float heightPixel = height * camHeight;

    camScene->addRect(xLeftPixel, yTopPixel, widthPixel, heightPixel, pen, brush);
}
