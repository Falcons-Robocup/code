// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MLALLOBJECTS_H
#define MLALLOBJECTS_H

#include "mlObject.hpp"
#include "dewarp.hpp"

#include <QVector>
#include <QWidget>
#include <QtGui>
#include <QGraphicsView>
#include <QGraphicsRectItem>


class mlAllObjects
{
public:
    mlAllObjects(QGraphicsScene *camScene = nullptr, QGraphicsScene *topScene = nullptr);
    void config(const int robot = -1);
    void push_back(const mlObject object) { allObjects.push_back(object); };
    void clear() { allObjects.clear(); };
    void drawCamScene(const uint width, const uint height, uint32_t frameId, quint64 mSecs, float fps = 0.0);
    void drawTopScene(const uint width, const uint height);
    QString getPrintString(bool fixedSize);
    void update();
    void print(uint frameId);

    QVector<mlObject> getAllObjects() { return allObjects; };

private:
    QVector<mlObject> allObjects;
    Dewarper *dewarp[4] = {nullptr, nullptr, nullptr, nullptr};
    QGraphicsScene *camScene = nullptr;
    QGraphicsScene *topScene = nullptr;

    float camWidth, camHeight, topWidth, topHeight;

    void drawCamObject(const mlObject obj);
    void drawTopObject(const mlObject obj);

    void addLineCamRel(const float x0, const float y0, const float x1, const float y1, const QPen pen);
    void addEllipseCamRelCenter(const float xCenter, const float yCenter, const float radius, const QPen pen);
    void addRectCamRelCenter(const float xCenter, const float yCenter, const float width, const float height, const QPen pen, const QBrush brush);

    void addLineTopRel(const float x0, const float y0, const float x1, const float y1, const QPen pen);
    void addLineTopAngle(const float x, const float y, const float angle, const float radius0, const float radius1, QPen pen);
    void addEllipseTopRelCenter(const float xCenter, const float yCenter, const float width, const float height, const QPen pen);
};

#endif // MLALLOBJECTS_HPP
