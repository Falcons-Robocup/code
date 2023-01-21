// Copyright 2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include <QObject>

#include "determinePosition.hpp"

class mainWidget; //forward declaration

class localization : public QObject {
   Q_OBJECT

public:
   explicit localization(mainWidget *parent);
   ~localization();

   detPosSt getCalc() { return calc; }
   std::vector<azimuthRadiusSt> getPixelList() { return  pixelList; }

private:
   mainWidget *parent = nullptr;
   detPosSt calc;

   std::vector<azimuthRadiusSt> pixelList;
   size_t printCounter;


public slots:
   void process(size_t cam);
   void stop();

signals:
   void finished(); // used to cleanup thread, is required because used as signal

};

#endif // LOCALIZATION_HPP
