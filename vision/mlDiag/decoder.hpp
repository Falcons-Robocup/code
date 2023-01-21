// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef DECODER_HPP
#define DECODER_HPP

#include "frame.hpp"

#include <QWidget>
#include <QVector>

struct objectSt {
    float azimuth; // -M_PI/2..M_PI/2 (-90..90 degrees), by definition 0 degrees is nort and 90 degrees is east
    quint8 classId; // 0..4, negative value indicates invalid class
    QColor color;
    float confidence; // 0..1 (range does not include 1.0 itself)
    float elevation; // -M_PI/2..M_PI/2 (-90..90 degrees)
    float height; // 0..1 (range does not include 1.0 itself)
    QString name;
    QString nameShort;
    float radius; // 0..MAX_FLT in meters
    float valid;
    float width; // 0..1 (range does not include 1.0 itself)
    float xCenter; // 0..1, 0 = left (range does not include 1.0 itself)
    float yCenter; // 0..1, 0 = top (range does not include 1.0 itself)
};

struct statSt {
    quint64 mSecs; // since Epoch in UTC
    quint32 frameId;
};

class decoder : public QWidget
{
    Q_OBJECT
public:
    decoder(QWidget *parent = nullptr);
    void update(frame frm);

    QString getPrintStory(const bool fixedSize);
    QVector<objectSt> getObjects() { return objList; };
    statSt getStatistics() { return stat; };

private:
    float topWidth, topHeight;

    QString getPrintString(const int index, const bool shortName);

    QVector<objectSt> objList;
    statSt stat;

};

#endif // DECODER_HPP
