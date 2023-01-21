// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MLOBJECT_HPP
#define MLOBJECT_HPP

#include "dewarp.hpp"

#include <QColor>

class mlObject {
public:
    mlObject();
    void update(Dewarper *dewarp);
    QString getPrintString(int index, bool shortName);
    void print(int index);

    float azimuth; // -M_PI/2..M_PI/2 (-90..90 degrees)
    int camera;
    int classId; // 0..4, negative value indicates invalid class
    float confidence; // 0..1 (range does not include 1.0 itself)
    float elevation; // -M_PI/2..M_PI/2 (-90..90 degrees)
    uint frameId;
    float height; // 0..1 (range does not include 1.0 itself)
    bool isDewarped;
    QString name;
    QString nameShort;
    float radius; // 0..MAX_FLT in meters
    bool valid;
    float width; // 0..1 (range does not include 1.0 itself)
    float xCenter; // 0..1 (range does not include 1.0 itself)
    float yCenter; // 0..1 (range does not include 1.0 itself)
    QColor color;

private:
    Dewarper *dewarp = nullptr;
    void doDewarp();

};

#endif // MLOBJECT_HPP
