// Copyright 2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "localization.hpp"

#include "mainWidget.hpp"


localization::localization(mainWidget *parent) {
   this->parent = parent;

   calc.confidence = 0.0;
   calc.goodEnough = false;
   printCounter = 0;
}


localization::~localization() {
   // free resources
}

void localization::process(size_t camera) {
   // qDebug("INFO    localization from %zu", camera);
   if( camera == 0 ) { // only perform the localization when camera 0 is grabbed
      // TODO: camera's should be synchronized
      // convert camera images to linepoints
      parent->lPoints->update(parent->rgbImages);

      // story a copy of the pixelList for the topViewer (which is running asynchronous to the localization
      pixelList = parent->lPoints->getPixelList();

      parent->detPos->pointsToPosition(); // perform the calculation

      calc = parent->detPos->getGoodEnoughLoc();
      // calc.pos.rz += 2.0 * M_PI * 45.0/360.0;

      if( printCounter > 10 ) {
         qDebug("INFO    on floor %3d off floor %3d tries %3d age %5d last %2d confidence %3.0f %%",
                calc.amountOnFloor, calc.amountOffFloor, calc.numberOfTries, calc.age, calc.lastActive, 100.0 * calc.confidence);
         printCounter = 0;
      } else {
         printCounter++;
      }

   }
}

void localization::stop() {
   qDebug("INFO    localization thread stop");
}
