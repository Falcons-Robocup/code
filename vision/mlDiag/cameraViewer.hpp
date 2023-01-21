// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CAMERAVIEWER_HPP
#define CAMERAVIEWER_HPP

#include <QGraphicsScene>
#include <QPen>
#include <QVector>
#include <QWidget>

#include "decoder.hpp"

class cameraViewer {
public:
    cameraViewer(QGraphicsScene *camScene);
    void drawCamScene(const uint width, const uint height, QVector<objectSt> objects, statSt stats);

private:
    void drawCamObject(const objectSt obj);
    void addLineCamRel(const float x0, const float y0, const float x1, const float y1, const QPen pen);
    void addEllipseCamRelCenter(const float xCenter, const float yCenter, const float radius, const QPen pen);
    void addRectCamRelCenter(const float xCenter, const float yCenter, const float width, const float height, const QPen pen, const QBrush brush);

    QGraphicsScene *camScene = nullptr;
    float camWidth, camHeight, topWidth, topHeight;
    QPen penGrid;

};

#endif // CAMERAVIEWER_HPP
