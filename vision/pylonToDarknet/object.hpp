// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef OBJECT_HPP
#define OBJECT_HPP

#include <QColor>
#include <QString>

#include "dewarp.hpp"

class object {
public:
   object();
   void update(int camera, Dewarper *dewarpLookup, int robot); // TODO rename to dewarp
   QString getPrintString(bool shortName);
   void print(bool shortname);

   float azimuth; // -M_PI/2..M_PI/2 (-90..90 degrees)
   int camera;
   int classId; // 0..4, negative value indicates invalid class
   float confidence; // 0..1 (range does not include 1.0 itself)
   float elevation; // -M_PI/2..M_PI/2 (-90..90 degrees)
   float height; // 0..1 (range does not include 1.0 itself)
   bool isDewarped;
   QString name;
   QString nameShort;
   float radius; // 0..MAX_FLT in meters
   int robot;
   bool valid;
   float width; // 0..1 (range does not include 1.0 itself)
   float xCenter; // 0..1 (range does not include 1.0 itself)
   float yCenter; // 0..1 (range does not include 1.0 itself)
   QColor color;

   Dewarper *dewarpLookup = nullptr;

private:
   void doDewarp();
   void rangeCheck();
};

#endif // OBJECT_HPP
