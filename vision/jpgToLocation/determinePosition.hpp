// Copyright 2014-2022 Andre Pool and Geraldo Santiago
// SPDX-License-Identifier: Apache-2.0

#ifndef DETERMINEPOSITION_HPP
#define DETERMINEPOSITION_HPP

#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>

#include "fieldLut.hpp"
#include "config.hpp"
#include "linePoints.hpp"
#include "robotFloor.hpp"

struct detPosSt {
   positionStDbl pos;
   bool goodEnough;
   float confidence;
   int lastActive; // last time (in frames) location was found
   int age; // life time in amount of frames
   int amountOnFloor;
   int amountOffFloor;
   int numberOfTries;
   bool operator<(const detPosSt &val) const {
      // sorting this struct is performed on confidence (higher is better) - lastActive (penalty) + age (improve)
      return (confidence - lastActive * 0.01f + age * 0.00001f) > (val.confidence - val.lastActive * 0.01f + val.age * 0.00001f);
   }
};

class determinePosition {

public:
   determinePosition(linePoints *lPoints, int ageThreshHold, robotFloor *rFloor);
   ~determinePosition();
   void pointsToPosition();
   std::vector<detPosSt> getLocList();

   detPosSt getGoodEnoughLoc(); // get the last know position or none
   detPosSt getGoodEnoughLocExport();
   void setManualMode(const float xOffset, const float yOffset, const float rz);


private:
   struct th {
      pthread_t thread;
      determinePosition *classContext;
      detPosSt determinedPosition;
      detPosSt start;
      detPosSt found;
      int id;
      int ret;
      double confidenceThreshHold;
      cv::Ptr<cv::optim::DownhillSolver> solverLUT;
      fieldLut *costTable;
   };

   // pointers for access to other classes
   int ageThreshHold;
   fieldLut *LUT_ptr;
   robotFloor *rFloor = nullptr;
   bool manualMode = true;

   double confidenceThreshHold;
   double sameLocRangePow2; // when locations are within this range it is expected that is the same range, pow2 to save an sqrt for each check
   std::vector<detPosSt> locList, locListExport;
   struct th threads[5]; // we have 4 cpu's (8 with hyperthreading) available for: 1 good position + 3 possible positions (which toggle sometimes with the good position) + 1 random position
   unsigned int numThreads;
   std::mutex exportMutex;

   bool locConfident; // active when found in recent past
   int lostLoc; // keep track when we lost lock when we recently where confident we had the position
   detPosSt goodEnoughLoc; // containing the last known position

   positionStDbl createRandomPosition(positionStDbl previousPos, bool useRecentPos);
   detPosSt optimizePosition(positionStDbl startPos, bool localSearch, cv::Ptr<cv::optim::DownhillSolver> solverLUT);
   static void* processOneLocation(void *id);
   void goodEnough(); // determine the best position (if any)
};

#endif
