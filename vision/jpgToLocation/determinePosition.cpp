// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "determinePosition.hpp"

#include <cmath> // fmod

determinePosition::determinePosition(linePoints *lPoints, int ageThreshHold, robotFloor *rFloor) {
   this->rFloor = rFloor;
   this->ageThreshHold = ageThreshHold;
   confidenceThreshHold = 0.825; // keep when more then 82.5 %
   sameLocRangePow2 = 8 * 8; // most distances (after solving) are within 1.0 of each other !

   // TODO: combine both below to one instance
   LUT_ptr = new fieldLut(lPoints, rFloor);
   fieldLut *costTable = new fieldLut(lPoints, rFloor);

   locConfident = false; // at start we do not know where we are
   lostLoc = INT_MAX; // watchdog to keep track how long we are waiting for a new good candidate
   goodEnoughLoc.confidence = 0.0; // default the goodEnough is invalid
   goodEnoughLoc.lastActive = INT_MAX;
   goodEnoughLoc.pos.x = 0.0;
   goodEnoughLoc.pos.y = 0.0;
   goodEnoughLoc.pos.rz = 0.0;

   manualMode = false;

   numThreads = sizeof(threads) / sizeof(threads[0]);

   // Create the solver
   // the default solver epsilon is 1e-06 and maxcount 5000
   for( unsigned int ii = 0; ii < numThreads; ii++ ) {
      threads[ii].solverLUT = cv::optim::createDownhillSolver();
      // connect the function to the solver
      threads[ii].solverLUT->setFunction(LUT_ptr);
      threads[ii].id = ii;
      threads[ii].classContext = this;
      threads[ii].costTable = costTable;
      threads[ii].confidenceThreshHold = confidenceThreshHold;
   }
}

determinePosition::~determinePosition() {
   // Properly remove created objects
   if( threads[0].costTable != NULL ) {
      delete threads[0].costTable;
      threads[0].costTable = NULL;
   }

   // Do not delete LUT_ptr since this object will be deleted by
   // the destructor of createDownhillSolver
}

void determinePosition::setManualMode(const float xOffset, const float yOffset, const float rz) {
   manualMode = true;
   if( locList.size() > 1 ) { // remove locations that are created in non manual mode
      locList.erase(locList.begin(), locList.begin() + locList.size());
   }

   if( locList.size() < 1 ) { // at least one location should be available for  manual mode
      detPosSt newPos;
      locList.push_back(newPos);
   }

   locList[0].pos.x = xOffset; // range 0.0 to 1.0
   locList[0].pos.y = yOffset; // range 0.0 to 1.0
   locList[0].pos.rz = rz; // range -Pi to Pi
}

detPosSt determinePosition::optimizePosition(positionStDbl startPos, bool localSearch,
                                             cv::Ptr<cv::optim::DownhillSolver> solverLUT) {
   detPosSt result;

   // if needed update the criteria maximum counter from the configuration interface
   cv::TermCriteria criteria = solverLUT->getTermCriteria();
   criteria.maxCount = 400; // TODO conf->getSolver().maxCount;
   solverLUT->setTermCriteria(criteria);

   // set the step size
   double xStep, yStep, rzStep;
   if( localSearch ) {
      // robot speed max 5ms max, minimal 5fps, so the move per frame is max 1m = 50 pixels (worst case this is 50 pixels x or 50 pixels y
      // robot speed max 2ms, minimal 10 FPS, so the move per frame is 0.20 meter
      // 0.20 / 20 meter =
      xStep = 0.01; // 0.20 meter / 20 meter
      yStep = 0.014; // 0.20 meter / 14 meter
      // one turn takes minimal 2 seconds, minimal 10 fps, 2 Pi degrees / (2 * 10)
      rzStep = 2 * M_PI / 20.0; // TODO is this a good value
   } else {
      xStep = 0.05; // TODO is this a good value
      yStep = 0.06; // TODO is this a good value
      rzStep = 2 * M_PI / 10.0; // TODO is this a good value
   }
   cv::Mat stepTF = (cv::Mat_<double>(3, 1) << xStep, yStep, rzStep);
   solverLUT->setInitStep(stepTF);

   // set the regions where x, y and rz can be chosen from
   double rzMin = 0.0f - rzStep; // this is not the compass range, but floor range
   // TODO could rzMax only Pi instead of 2 Pi (field is symetrical), probably not, becuase we don't want a flip
   double rzMax = 2 * M_PI + rzStep; // TODO: why subrackt and add rzStep?, is this to have some margin around 0 degrees ??
   // the robot can be anywhere on the floor, so also on the border, which is outside the field
   // this means from the upper left corner (0,0) to the lower right corner
   double minConst[] = { 0.0, 0.0, rzMin };
   double maxConst[] = { 1.0, 1.0, rzMax };

   solverLUT->setMinConstraint(minConst, 3);
   solverLUT->setMaxConstraint(maxConst, 3);

   // create matrix with starting point for the solver algorithm
   // the step size is influencing the center of solver, it looks like it is subtracted from the given position
   // so the given position is not the center, but the "upper" border.
   // convert the known center position to the "upper" border, so the solver will effective start from the known center position
   double upperX = startPos.x; // + conf->getSolver().xStep/2;
   double upperY = startPos.y; // + conf->getSolver().yStep/2;
   double upperRz = startPos.rz; // + conf->getSolver().rzStep/2;

   cv::Mat xTF = (cv::Mat_<double>(1, 3) << upperX, upperY, upperRz);
   // start the solver with this starting point
   result.confidence = 1.0 - solverLUT->minimize(xTF); // solver will return lowest value, the position will be in xTF
   result.numberOfTries = solverLUT->numberOfTries();
   // So now the algorithm is done, it might have found the final point, or it maybe stuck in a local minimum

   // get the found position
   result.pos.x = xTF.at<double>(0, 0);
   result.pos.y = xTF.at<double>(0, 1);
   result.pos.rz = xTF.at<double>(0, 2);

   // print warning when result out of range (should never happen)
   if( result.pos.x < 0.0 || result.pos.x > 1.0 ) {
      std::cout << "solver x  out of range: " << result.pos.x << std::endl;
   }
   if( result.pos.y < 0.0f || result.pos.y > 1.0 ) {
      std::cout << "solver y  out of range: " << result.pos.y << std::endl;
   }
   if( result.pos.rz >= rzMin && result.pos.rz < 0 ) {
      result.pos.rz = result.pos.rz + 2 * M_PI;
   } else if( result.pos.rz >= 2 * M_PI && result.pos.rz <= rzMax ) {
      result.pos.rz = result.pos.rz - 2 * M_PI;
   } else if( result.pos.rz < rzMin || result.pos.rz > rzMax ) {
      std::cout << "solver rz out of range: " << result.pos.rz << std::endl;
   }
   return result;
}

void *determinePosition::processOneLocation(void *id) {
   struct th *context = (struct th*)id;
   determinePosition *cls = context->classContext;

   bool localSearch = context->start.confidence > context->confidenceThreshHold;
   context->found = cls->optimizePosition(context->start.pos, localSearch, context->solverLUT);
   // done with the search
   context->found.lastActive = 0; // the result was just found so lastActive = 0
   context->found.age = context->start.age + 1; // this result is derived from ii, so increment the age of ii by 1
   if( context->found.confidence > context->confidenceThreshHold ) {
      // for statistics we also like to get the amountOnFloor and amountOffFloor (we have already position and confidence)
      // so run the costTable calculation once again with the known position
      float x[3];
      x[0] = context->found.pos.x; // range 0.0 to 1.0
      x[1] = context->found.pos.y; // range 0.0 to 1.0
      x[2] = context->found.pos.rz; // range 0.0 to 2 Pi (solver returns in range 0.0 to 2 Pi)

      scoreStruct tmp = context->costTable->calcSimple(x);
      context->found.amountOffFloor = tmp.amountOffFloor;
      context->found.amountOnFloor = tmp.amountOnFloor;
      if( context->found.confidence != tmp.confidence ) {
         std::cout
               << "multiCam: Determine Position Warning: recalculation of confidence differs when re-calculating with best position, solver "
               << context->found.confidence << " recalculation " << tmp.confidence << std::endl;
         // might be related to criteria.maxCount, when reducing this in gui to e.g. 40, the error appears very often
      }
   }

   // APOX: check below paranoia mode, can be removed
   if( context->found.confidence >= 0.0f && context->found.confidence <= 1.0f ) { // discard when confidence is not within range 0 to 1, e.g. nan
      // all fine
   } else {
      std::cout
            << "multiCam: Determine Position Error: confidence out range, should never happen because already checked before"
            << context->found.confidence << std::endl;
      context->found.confidence = 0.0; // reject result
      exit(EXIT_FAILURE);
   }
   return 0; // pointer instead of void
}

// function to create a random point on the field
positionStDbl determinePosition::createRandomPosition(positionStDbl recentPos, bool useRecentPos) {
   positionStDbl pos;
   bool nearByPoint = true;
   // int numTries = 0;
   pos.rz = M_PI * ((double)rand()/(double)RAND_MAX); // field is symmetrical, only half the search area required
   do {
      nearByPoint = false;
      pos.x = (double)rand()/(double)RAND_MAX; // robot can be anywhere on the floor (so also outside the field)
      pos.y = (double)rand()/(double)RAND_MAX;
      if( recentPos.x > 100000 ) {
         printf("dummy to suppress compile warning about not used parameter\n");
      }
   } while( nearByPoint && useRecentPos ); // we only need the nearby check if the location was not found in the previous frame

   return pos;
}

void determinePosition::pointsToPosition() {
   if( manualMode ) {
      // calculate the confidence from the proviced position
      float x[3];
      x[0] = locList[0].pos.x; // range 0.0 to 1.0
      x[1] = locList[0].pos.y; // range 0.0 to 1.0
      x[2] = locList[0].pos.rz; // range -Pi to Pi instead of 0.0 to 2 Pi

      scoreStruct tmp = threads[0].costTable->calcSimple(x);
      // done with calculation, return the single position
      locList[0].confidence = tmp.confidence;
      locList[0].amountOffFloor = tmp.amountOffFloor;
      locList[0].amountOnFloor = tmp.amountOnFloor;
      locList[0].numberOfTries = 0;
      locList[0].age = 0; // need lower then threshold for visualization
      locList[0].lastActive = 0;
      locList[0].goodEnough = true;
   } else {
      // reduce the locList size to the available thread positions (numThreads)
      // subtract one more location, so we can add the random position
      if( locList.size() >= (numThreads - 1) ) {
         locList.erase(locList.begin() + (numThreads - 1), locList.begin() + locList.size());
      }

      // create a new random position
      detPosSt randomLoc;
      positionStDbl zeroLoc;
      zeroLoc.x = 0.0;
      zeroLoc.y = 0.0;
      zeroLoc.rz = 0.0;
      randomLoc.pos = createRandomPosition(zeroLoc, 0); // APOX todo, randomPosition should not be close to one of the found positions
      randomLoc.confidence = 0.0; // this new random location always is thrown out, depending on the solver result, the new found position might be kept
      randomLoc.lastActive = 0;
      randomLoc.age = 0;
      locList.push_back(randomLoc);

      // now run the position detection algorithm on all previous found positions, including one new random location
      size_t locSize = locList.size(); // use intermediate variable, because the size changes during the loop

      // run for each input location the optimization process
      void *ret;

      // double check if the amount of position searches fits in the available threads
      if( locSize > numThreads ) {
         std::cout << "ERROR    Determine Position not enough threads to run all position searches" << std::endl;
         exit(EXIT_FAILURE);
      }

      // start the searches in different threads
      for( size_t ii = 0; ii < locSize; ii++ ) {
         threads[ii].start = locList[ii];
         if( pthread_create(&(threads[ii].thread), NULL, processOneLocation, &(threads[ii])) ) {
            std::cout << "ERROR   Determine Position cannot creating threads" << std::endl;
            exit(EXIT_FAILURE);
         }
      }

      // wait until all threads are finished
      for( size_t ii = 0; ii < locSize; ii++ ) {
         pthread_join(threads[ii].thread, &ret);
         if( threads[ii].found.confidence > confidenceThreshHold ) {
            locList.push_back(threads[ii].found);
         }
         // update statistics old location, used in case derived position worse
         // if derived location better then this old location will be removed in the distance check
         locList[ii].lastActive++; // this location is not updated in this frame, increase last active moment
         locList[ii].age++; // location exist one frame longer
      }

      // check if positions are the same, if so, reject the one with the worst confidence (by setting confidence to 0.0)
      for( size_t ii = 0; ii < locList.size() - 1; ii++ ) {
         for( size_t jj = ii + 1; jj < locList.size(); jj++ ) {
            // std::cout << "ii: " << ii << " jj: " << jj << std::endl;
            double x = locList[ii].pos.x - locList[jj].pos.x;
            double y = locList[ii].pos.y - locList[jj].pos.y;
            // TODO: add the rz also in this equation, because for the same location delta rz shall also be small
            double distancePow2 = x * x + y * y;

            if( distancePow2 < sameLocRangePow2 ) {
               double iiConfidence = locList[ii].confidence - locList[ii].lastActive * 0.01f; // confidence also depends on when last found, older locations get a penalty, that will be significant after 20 frames
               double jjConfidence = locList[jj].confidence - locList[jj].lastActive * 0.01f;
               if( iiConfidence > jjConfidence ) {
                  locList[jj].confidence = 0.0; // ii better, reject jj
               } else {
                  locList[ii].confidence = 0.0; // jj better, reject ii
               }
               // use the oldest age, independent which had a better confidence
               if( locList[ii].age < locList[jj].age ) {
                  locList[ii].age = locList[jj].age;
               } else {
                  locList[jj].age = locList[ii].age;
               }
               // std::cout << "same location: distance " << sqrt(distancePow2) << ", age " << locList[jj].age << ", loc " << locList[ii].pos.x << " " << locList[ii].pos.y << ", " << locList[jj].pos.x << " " << locList[jj].pos.y << std::endl;
            } else {
               // because the field is symmetrical, also check if we are looking at the mirror point, if so, this can be treated as the same point
               x = (1.0 - locList[ii].pos.x) - locList[jj].pos.x;
               y = (1.0 - locList[ii].pos.y) - locList[jj].pos.y;
               // TODO: add the rz also in this equation, because for the same location delta rz shall also be small
               distancePow2 = x * x + y * y;
               if( distancePow2 < sameLocRangePow2 ) {
                  double iiConfidence = locList[ii].confidence - locList[ii].lastActive * 0.01f; // confidence also depends on when last found, older locations get a penalty, that will be significant after 20 frames
                  double jjConfidence = locList[jj].confidence - locList[jj].lastActive * 0.01f;
                  if( iiConfidence > jjConfidence ) {
                     locList[jj].confidence = 0.0; // ii better, reject jj

                     // preserve the oldest lock
                     if( locList[ii].age < locList[jj].age ) {
                        locList[ii].age = locList[jj].age; // so jj was older but ii better, keep the same floor half of the oldest loc
                        // this means we have to mirror ii to the other side of the floor, also rz of ii has to be mirrored
                        locList[ii].pos.x = 1.0 - locList[ii].pos.x;
                        locList[ii].pos.y = 1.0 - locList[ii].pos.y;
                        if( locList[ii].pos.rz > M_PI ) {
                           locList[ii].pos.rz -= M_PI;
                        } else {
                           locList[ii].pos.rz += M_PI;
                        }
                     }
                  } else {
                     locList[ii].confidence = 0.0; // jj better, reject ii

                     // preserve the oldest lock
                     if( locList[ii].age > locList[jj].age ) {
                        locList[jj].age = locList[ii].age; // so ii was older but jj better, keep the same floor half of the oldest loc
                        // this means we have to mirror jj to the other side of the floor, also rz of jj has to be mirrored
                        locList[jj].pos.x = 1.0 - locList[jj].pos.x; // but we now use the mirror point
                        locList[jj].pos.y = 1.0 - locList[jj].pos.y;
                        if( locList[jj].pos.rz > M_PI ) {
                           locList[jj].pos.rz -= M_PI;
                        } else {
                           locList[jj].pos.rz += M_PI;
                        }
                     }
                  }
               }
               // std::cout << "same mirror location: distance " << sqrt(distancePow2) << ", age " << locList[jj].age << ", loc " << locList[ii].pos.x << " " << locList[ii].pos.y << ", " << locList[jj].pos.x << " " << locList[jj].pos.y << std::endl;
            }
         }
      }

      // remove locations that are not required anymore (in which case the confidence is set to 0.0)
      // running reverse through the index, because i don't know what happens with the index to a higher element when a lower element was removed
      for( int ii = (int)locList.size() - 1; ii >= 0; ii-- ) { // use index type int because the check stops when negative
         if( locList[ii].confidence <= 0.0 ) {
            // cout << "erase  ii " << ii << ", size " << locList.size();
            locList.erase(locList.begin() + ii);
            // cout << ", again size " << locList.size() << endl;
         }
      }

      // sort list on confidence, highest confidence first
      std::sort(locList.begin(), locList.end());
   }
   exportMutex.lock();
   locListExport = locList;
   exportMutex.unlock();

   goodEnough();

   if( locList.size() > 0 ) {
      // send new position to WorldModel
      // TODO: send list candicates to WorldModel
   }
}

std::vector<detPosSt> determinePosition::getLocList() {
   exportMutex.lock();
   printf("WARNING use goodEnoughLoc instead\n");
   std::vector<detPosSt> retVal = locListExport;
   exportMutex.unlock();
   return retVal;
}

// find the a valid candidate from the locList (if any)
// there are two states
// 1. in the resent past we found a valid candidate (locConfident == true)
// 2. in the resent past we have not found a valid candidate (locConfident == false)

// state 1 (locConfident == true)
// if we found a valid candidate in the past (confidentPosition) we want to check
// if any of the NEW candidates is close to the previous known candidate (small xDelta, yDelta and rzDelta)
// then take that candidate
// if there are more candidates nearby, choose the one closest to the previous known candidate
// also check the mirrored candidates (because the field is symmetrical)
// when none of the new candidates is close then increase the lostLoc and do not provide a new position candidate
// when the locLost exceeds a threshold switch to state 2 (locConfident == false)

// state 2 (locConfident == false)
// we have no clue where we are
// now search the candidate list for a NEW candidate with a very good confidence that exist for a while (age)
// if such a candidate is available, take that candidate, otherwise do not provide a new position candidate

void determinePosition::goodEnough() {
   struct candidateSelectorSt {
      double error; // combined error of multiple variables e.g. xDelta, confidence, ..
      double xDelta; // TODO: remove next 3 variables
      double yDelta;
      double rzDelta;
      size_t index; // index that points to the candidate in the locList
      bool mirror; // use the position of the candidate or the mirrored position of the candidate
      bool operator<(const candidateSelectorSt &val) const {
         // sorting this struct is performed on error (lowest value first)
         return error < val.error;
      }
   };

   // create a list that contains indexes for locList with possible candidates for the position
   std::vector<candidateSelectorSt> candSelList;

   if( locConfident ) {
      // so we are pretty sure where we are, try to find the best candidate in the list of NEW list that is close to the previous candidate
      for( size_t ii = 0; ii < locList.size(); ii++ ) { // iterate through candidates
         if( locList[ii].lastActive == 0 ) { // only use when new
            for( size_t jj = 0; jj < 2; jj++ ) { // normal and mirrored location
               candidateSelectorSt tmp;
               double xDelta, yDelta, rzDelta;
               tmp.index = ii;
               if( jj == 0 ) {
                  xDelta = abs(goodEnoughLoc.pos.x - locList[ii].pos.x);
                  yDelta = abs(goodEnoughLoc.pos.y - locList[ii].pos.y);
                  rzDelta = fmod((goodEnoughLoc.pos.rz - locList[ii].pos.rz + 2 * M_PI), 2 * M_PI); // always positive (range 0 to 2 Pi)
                  tmp.mirror = false;
               } else {
                  // mirror
                  xDelta = abs(goodEnoughLoc.pos.x - (1.0 - locList[ii].pos.x));
                  yDelta = abs(goodEnoughLoc.pos.y - (1.0 - locList[ii].pos.y));
                  rzDelta = fmod((goodEnoughLoc.pos.rz - locList[ii].pos.rz + M_PI + 2 * M_PI), 2 * M_PI); // always positive (range 0 to 2 Pi)
                  tmp.mirror = true;
               }
               if( rzDelta > M_PI ) {
                  rzDelta = 2 * M_PI - rzDelta; // determine the disance from 2 Pi instead of 0
               } // an rz delta of e.g. (2 Pi - 1) is the same as rz delta of 1 (it is about the distance from 0 or 2 Pi)
               tmp.xDelta = xDelta;
               tmp.yDelta = yDelta;
               tmp.rzDelta = rzDelta;
               // the next error only used to sort the canditates, the threshold is applied separate for x, y and rz
               tmp.error = xDelta + yDelta + rzDelta/(2*M_PI); // assume one full rotation is the same as the robot on the other side field
               candSelList.push_back(tmp);
            }
         }
      }
   } else {
      // so we have no clue about the current location, try to find one
      // it should be new, exists for a while and have a good confidence
      for( size_t ii = 0; ii < locList.size(); ii++ ) {
         if( (locList[ii].lastActive == 0) && (locList[ii].age > ageThreshHold ) ) { // only new candidates that exist for at least one second
            // combine the confidence and age to a decision variable (higher is better)
            // which requires scaling of the age in the confidence range (which is in the order of 0.1)
            // (log(1000) - log(1)) / 10  = 0.69
            // (log(1000) - log(10)) / 10 = 0.46
            // (log(1000) - log(100)) / 10 = 0.23
            // (log(1000) - log(200)) / 10 = 0.16
            // (log(1000) - log(500)) / 10 = 0.069
            // (log(1000) - log(1000)) / 10 = 0
            // log returns natural logarithm (2.73 instead of e.g. log10)
            if( locList[ii].age < 1 ) {
               // TODO: check if age is starting at 0, if so, increase by 1
               // TODO: remove this check
               printf("determinePosition ERROR: age is %d, which is < 1\n", locList[ii].age);
            }
            candidateSelectorSt tmp;
            tmp.index = ii;
            tmp.error = (log(1000) - log(locList[ii].age)) / 10.0; // a bad age results in > 0.2 a good age results in < 0.05
            if( tmp.error < 0 ) {
               tmp.error = 0;
            }
            tmp.error += (1.0 - locList[ii].confidence); // combine age and confidence
            if( tmp.error > 1.0 ) {
               tmp.error = 1.0;
            } // truncate in case a very short age and bad confidence
            candSelList.push_back(tmp);
         }
      }
   } // locConfident

   // now we have the indexes to the locList with possible candidates
   // sort this list (on the error), to get the best candidate
   sort(candSelList.begin(), candSelList.end());
   // the first element in the list (lowest error) contains the index to the locList with the best candidate

   // use this index to get the position from the locList and mirror variable to determine if this positions shall be mirrored
   // now check if the candidate is good enough (has good confidence and available for a while)
   // note: the error value has a different meaning for locConfident == true state and locConfident == false state
   exportMutex.lock();
   goodEnoughLoc.goodEnough = false; // default the location is not good enough
   if( lostLoc != INT_MAX ) {
      lostLoc++; // default we do not have lock, and increase watchdog and make goodEnough.lastActive higher
   }
   if( locConfident ) {
      // so we had a valid position in the recent past
      if( candSelList.size() >= 1 ) {
         // convert from meters to relative (0.0 to 1.0)
         float xDeltaThreshold = 0.65 * rFloor->getXScale(); // TODO conf->getGoodEnough().xDeltaKeepThreshold
         float yDeltaThreshold = 0.65 * rFloor->getYScale(); // TODO conf->getGoodEnough().yDeltaKeepThreshold
         float rzDeltaThreshold = 2.0 * M_PI * 40.0 / 360.0; // TODO conf->getGoodEnough().rzDeltaKeepThreshold;

         if( (candSelList[0].xDelta < xDeltaThreshold) && (candSelList[0].yDelta < yDeltaThreshold)
             && (candSelList[0].rzDelta < rzDeltaThreshold) ) {
            // so the new candidate is close to the previous known location
            lostLoc = 0; // reset the know where we are watchdog
            goodEnoughLoc = locList[candSelList[0].index];
            goodEnoughLoc.goodEnough = true;
            if( candSelList[0].mirror ) {
               // use the mirrored version of this location
               goodEnoughLoc.pos.x = 1.0 - goodEnoughLoc.pos.x;
               goodEnoughLoc.pos.y = 1.0 - goodEnoughLoc.pos.y;
               if( goodEnoughLoc.pos.rz > M_PI ) {
                  goodEnoughLoc.pos.rz -= M_PI;
               } else {
                  goodEnoughLoc.pos.rz += M_PI;
               }
            }
         }
      }
      if( lostLoc > 120 ) { // TODO conf->getGoodEnough().keepWatchdog ) { // 30fps * 4 = 4 seconds
         locConfident = false;
      }
   } else {
      // so we did not have a candidate in the past, lets checkout if one of the new ones is good enough (if any)
      for( size_t ii = 0; ii < candSelList.size(); ii++ ) { // Note: at this stage there can only be 0 or 1 candidate
         // so there is a candidate
         // printf(" score %f, age %d", locList[candSelList[ii].index].score, locList[candSelList[ii].index].age);
         if( candSelList[ii].error < 0.40 ) { // TODO conf->getGoodEnough().newThreshold
            // the candidate is good enough
            // use the "very old" position to select the candidate or mirrored candidate
            float xDelta = abs(goodEnoughLoc.pos.x - locList[candSelList[ii].index].pos.x);
            float yDelta = abs(goodEnoughLoc.pos.y - locList[candSelList[ii].index].pos.y);
            float rzDelta = fmod(goodEnoughLoc.pos.rz - locList[candSelList[ii].index].pos.rz + 2 * M_PI, 2 * M_PI); // range 0 to Pi
            if( rzDelta > M_PI ) {
               rzDelta = 2 * M_PI - rzDelta; // distance from 2 Pi
            } // an rz delta of e.g. (2 Pi - 1 ) is the same as rz delta of 1 (it is about the distance from 0.0 or 2 Pi)
            float errorStraight = xDelta + yDelta + rzDelta / (2.0 * M_PI); // assume 2 Pi degrees is the same as the robot to other side of the field (x or y error)
            // calculate the same for the mirrored position
            xDelta = abs(goodEnoughLoc.pos.x - (1.0 - locList[candSelList[ii].index].pos.x));
            yDelta = abs(goodEnoughLoc.pos.y - (1.0 - locList[candSelList[ii].index].pos.y));
            rzDelta = fmod(goodEnoughLoc.pos.rz - locList[candSelList[ii].index].pos.rz + 2 * M_PI + M_PI, 2 * M_PI); // range 0 to Pi
            if( rzDelta > M_PI ) {
               rzDelta = 2 * M_PI - rzDelta;
            } // range from -Pi to Pi
            double errorMirror = xDelta + yDelta + rzDelta / (2.0 * M_PI); // assume 2 Pi degrees is the same as the robot to other side of the field (x or y error)

            // select the one that is closest to the "very old" known position
            if( errorMirror < errorStraight ) {
               candSelList[ii].mirror = true;
            } else {
               candSelList[ii].mirror = false;
            }

            goodEnoughLoc = locList[candSelList[ii].index];
            goodEnoughLoc.goodEnough = true;
            if( candSelList[ii].mirror ) {
               // use the mirrored version of this location
               goodEnoughLoc.pos.x = 1.0 - goodEnoughLoc.pos.x;
               goodEnoughLoc.pos.y = 1.0 - goodEnoughLoc.pos.y;
               if( goodEnoughLoc.pos.rz > M_PI ) {
                  goodEnoughLoc.pos.rz -= M_PI;
               } else {
                  goodEnoughLoc.pos.rz += M_PI;
               }
            }
            // we know where we are, change to the other state
            locConfident = true; // go to other state where we know where we are
            lostLoc = 0; // valid location, reset the watchdog
         } // candSelList[ii].error
      } // candSelList.size()
   } // locConfident
   goodEnoughLoc.lastActive = lostLoc;
   exportMutex.unlock();
   //   printf("INFO    watch %4d sort candidates:", lostLoc);
   //   for( size_t ii = 0; ii < candSelList.size(); ii++ ) {
   //      printf(" %6.3f", candSelList[ii].error );
   //   }
   //   printf("\n");
}

detPosSt determinePosition::getGoodEnoughLoc() {
   exportMutex.lock();
   detPosSt retVal;
   if( manualMode ) {
      retVal = locListExport[0];
   } else {
      retVal = goodEnoughLoc;
   }
   exportMutex.unlock();
   return retVal;
}

detPosSt determinePosition::getGoodEnoughLocExport() {
   detPosSt retVal = getGoodEnoughLoc();
   // x should not be set outside the field
   if( retVal.pos.x < 0 ) {
      printf("ERROR   position x of %.1f is out of field, it should be higher then 0.0\n",
             retVal.pos.x);
      retVal.pos.x = 0.0;
   }
#ifdef NONO
   double xFieldEnd = (double)(rFloor->getWidth() - 1);
   if( retVal.pos.x > xFieldEnd ) {
      printf("ERROR   position x of %.1f is out of field, it should be lower then %.1f\n",
             retVal.pos.x, xFieldEnd);
      retVal.pos.x = xFieldEnd;
   }

   // y should not be set outside the field
   if( retVal.pos.y < 0 ) {
      printf("ERROR   position y of %.1f is out of field, it should be higher then 0.0\n",
             retVal.pos.y);
      retVal.pos.y = 0.0;
   }
   double yFieldEnd = (double)(rFloor->getHeight() - 1);
   if( retVal.pos.y > yFieldEnd ) {
      printf("ERROR   position y %.1f is out of field, it should be lower then %.1f\n",
             retVal.pos.y, yFieldEnd);
      retVal.pos.y = yFieldEnd;
   }
#endif
   // normalize to prevent issues when running of range for remote viewer export
   retVal.pos.rz = fmod(2 * M_PI + retVal.pos.rz, 2 * M_PI); // in radians, range 0 to 2 Pi
   return retVal;
}


