// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef ROBOTFLOOR_HPP
#define ROBOTFLOOR_HPP

#include <QGraphicsScene>

class robotFloor
{
public:
    robotFloor(QGraphicsScene *topScene);
    // void draw();
    void update(quint16 width, quint16 height);
    void addEllipse(const float xCenter, const float yCenter, const float width, const float height, QPen pen, QBrush brush);
    void addRect(const float xCenter, const float yCenter, const float width, const float height, QPen pen, QBrush brush);
    void addLine(const float x0, const float y0, const float x1, const float y1, QPen pen);
    void addLineAngle(const float x, const float y, const float angle, const float radius0, const float radius1, QPen pen);

    float getXScale() { return xScale; }
    float getYScale() { return yScale; }
    void addTopViewText(quint64 captureTime);
    void addText(const float x, const float y, const QString text, const int size = 12, const QColor color = Qt::white);

private:

    void addEllipseRelCenter(const float xCenter, const float yCenter, const float width, const float height, const QPen pen, const QBrush brush);
    void addLineRel(const float x0, const float y0, const float x1, const float y1, const QPen pen);
    void addRectRelCenter(const float xCenter, const float yCenter, const float width, const float height, const QPen pen, const QBrush brush);
    void addRectRel(const float xLeft, const float xRight, const float yTop, const float yBottom, QPen pen, const QBrush brush);
    void addArcRel(const float xBegin, const float yBegin, const float xRadius, const float yRadius, const float angleStart, const float angleDelta, const QPen pen);

    typedef struct {
        float xLeft;
        float xRight;
        float yTop;
        float yBottom;
    } floorSizeSt;

    typedef struct {
        float xLeft;
        float xMiddle;
        float xRight;
        float yTop;
        float yMiddle;
        float yBottom;
    } fieldSizeStruct;

    typedef struct {
        float left;
        float right;
        float mark;
    } penaltyXSizeStruct;

    typedef struct {
        float xRadius;
        float yRadius;
    } arcStruct;

    typedef struct {
        penaltyXSizeStruct xLeftArea;
        penaltyXSizeStruct xRightArea;
        float yTop;
        float yBottom;
        arcStruct arc;
    } penaltySizeStruct;

    typedef struct {
        float left;
        float right;
    } goalXSizeStruct;

    typedef struct {
        goalXSizeStruct xLeftArea;
        goalXSizeStruct xRightArea;
        float yTop;
        float yBottom;
        float xDepth; // APOX todo: remove
        float yPoleToCenter; // APOX todo: remove
    } goalSizeStruct;

    float topWidth, topHeight;
    QGraphicsScene *topScene = nullptr;
    floorSizeSt floorSize;
    fieldSizeStruct fieldSize;
    penaltySizeStruct penaltySize;
    goalSizeStruct goalSize;
    arcStruct cornerArc;
    arcStruct centerArc;
    float xScale;
    float yScale;

};

#endif // ROBOTFLOOR_HPP
