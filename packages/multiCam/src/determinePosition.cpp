 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "determinePosition.hpp"
#ifndef NOROS
#include "tracing.hpp"
#endif

using namespace cv;
using namespace std;

determinePosition::determinePosition(configurator *conf, linePointDetection *linePoint, preprocessor *prep,
        robotFloor *rFloor) {
    this->conf = conf;
    this->linePoint = linePoint;
    this->prep = prep;
    this->rFloor = rFloor;

    scoreThreshHold = conf->getScoreThresHold();
    sameLocRangePow2 = 8 * 8; // most distances (after solving) are within 1.0 of each other !

    // APOX todo: combine both below to one instance
    LUT_ptr = new fieldLut(conf, linePoint, rFloor);
    fieldLut *costTable = new fieldLut(conf, linePoint, rFloor);

    locConfident = false; // at start we do not know where we are
    lostLoc = INT_MAX; // watchdog to keep track how long we are waiting for a new good candidate
    goodEnoughLoc.score = 1.0; // default the goodEnough is invalid
    goodEnoughLoc.lastActive = INT_MAX;
    goodEnoughLoc.pos.x = 0.0;
    goodEnoughLoc.pos.y = 0.0;
    goodEnoughLoc.pos.rz = 0.0;

    numThreads = sizeof(threads) / sizeof(threads[0]);

    // Create the solver
    // the default solver epsilon is 1e-06 and maxcount 5000
    for (unsigned int ii = 0; ii < numThreads; ii++) {
        threads[ii].solverLUT = cv::optim::createDownhillSolver();
        // connect the function to the solver
        threads[ii].solverLUT->setFunction(LUT_ptr);
        threads[ii].id = ii;
        threads[ii].classContext = this;
        threads[ii].costTable = costTable;
        threads[ii].scoreThreshHold = scoreThreshHold;

    }
}

determinePosition::~determinePosition() {
    // Properly remove created objects
    if (threads[0].costTable != NULL) {
        delete threads[0].costTable;
        threads[0].costTable = NULL;
    }

    /* Do not delete LUT_ptr since this object will be deleted by
     * the destructor of createDownhillSolver
     */
}

detPosSt determinePosition::optimizePosition(positionStDbl startPos, bool localSearch,
        cv::Ptr<cv::optim::DownhillSolver> solverLUT) {
    detPosSt result;

    // if needed update the criteria maximum counter from the configuration interface
    TermCriteria criteria = solverLUT->getTermCriteria();
    criteria.maxCount = conf->getSolver().maxCount;
    solverLUT->setTermCriteria(criteria);

    // set the step size
    double xStep, yStep, rzStep;
    if (localSearch) {
        // robot speed max 5ms max, minimal 5fps, so the move per frame is max 1m = 50 pixels (worst case this is 50 pixels x or 50 pixels y
        xStep = 50.0f;
        yStep = 50.0f;
        rzStep = 40.0f; // 10 was not enough in case the robot rotates really fast
    } else {
        xStep = conf->getSolver().xStep;
        yStep = conf->getSolver().yStep;
        rzStep = conf->getSolver().rzStep;
    }
    cv::Mat stepTF = (cv::Mat_<double>(3, 1) << xStep, yStep, rzStep);
    solverLUT->setInitStep(stepTF);

    // set the regions where x, y and rz can be chosen from
    double rzMin = 0.0f - rzStep; // this is not the compass range, but floor range
    double rzMax = 360.0f + rzStep;
    // the robot can be anywhere on the floor, so also on the border, which is outside the field
    // this means from the upper left corner (0,0) to the lower right corner
    double minConst[] = { 0, 0, rzMin };
    double maxConst[] = { (double) rFloor->getXFloorRight(), (double) rFloor->getYFloorBottom(), rzMax };

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
    result.score = solverLUT->minimize(xTF); // the found point will be in xTF
    result.numberOfTries = solverLUT->numberOfTries();
    // So now the algorithm is done, it might have found the final point, or it maybe stuck in a local minimum

    // get the found position
    result.pos.x = xTF.at<double>(0, 0);
    result.pos.y = xTF.at<double>(0, 1);
    result.pos.rz = xTF.at<double>(0, 2);

    // print warning when result out of range (should never happen)
    if (result.pos.x < 0.0f || result.pos.x > (double) rFloor->getXFloorRight()) {
        cout << "solver x  out of range: " << result.pos.x << endl;
    }
    if (result.pos.y < 0.0f || result.pos.y > (double) rFloor->getYFloorBottom()) {
        cout << "solver y  out of range: " << result.pos.y << endl;
    }
    if (result.pos.rz >= rzMin && result.pos.rz < 0) {
        result.pos.rz = result.pos.rz + 360.0f;
    } else if (result.pos.rz >= 360.0f && result.pos.rz <= rzMax) {
        result.pos.rz = result.pos.rz - 360.0f;
    } else if (result.pos.rz < rzMin || result.pos.rz > rzMax) {
        cout << "solver rz out of range: " << result.pos.rz << endl;
    }
    return result;
}

void* determinePosition::processOneLocation(void *id) {
    struct th *context = (struct th*) id;
    determinePosition *cls = context->classContext;

    bool localSearch = context->start.score < context->scoreThreshHold;
    context->found = cls->optimizePosition(context->start.pos, localSearch, context->solverLUT);
    context->found.lastActive = 0; // the result was just found so lastActive = 0
    context->found.age = context->start.age + 1; // this result is derived from ii, so increment the age of ii by 1
    if (context->found.score < context->scoreThreshHold) {
        // for statistics we also like to get the amountOnFloor and amountOffFloor (we have already position and score)
        // so run the score calculation again with the known position
        scoreStruct tmp = context->costTable->calcSimple(context->found.pos, context->scoreThreshHold);
        context->found.amountOffFloor = tmp.amountOffFloor;
        context->found.amountOnFloor = tmp.amountOnFloor;
        if (context->found.score != tmp.score) {
            cout
                    << "multiCam: Determine Position Warning: recalculation of score differs when re-calculating with best position, solver "
                    << context->found.score << " recalculation " << tmp.score << endl;
            // might be related to criteria.maxCount, when reducing this in gui to e.g. 40, the error appears very often
        }
    }

    // APOX: check below paranoia mode, can be removed
    if (context->found.score >= 0.0f && context->found.score <= 1.0f) { // discard when score is not within range 0 to 1, e.g. nan
    // all fine
    } else {
        cout
                << "multiCam: Determine Position Error: score out range, should never happen because already checked before"
                << context->found.score << endl;
        context->found.score = 1.0f; // reject result
    }
    return 0;
}

// function to create a random point on the field
positionStDbl determinePosition::createRandomPosition(positionStDbl recentPos, bool useRecentPos) {
    positionStDbl pos;
    bool nearByPoint = true;
//    int numTries = 0;
    pos.rz = (double) (rand() % 180); // field is symmetrical, only half the search area required
    do {
        nearByPoint = false;
        pos.x = rand() % rFloor->getXFloorRight(); // robot can be anywhere on the floor (so also outside the field)
        pos.y = rand() % rFloor->getYFloorBottom();
#ifdef NONO
        // APOX: if we discard close points we might end up in a situation that a wrong found location
        // might block the nearby correct location that is e.g. rotated 90 degrees
        // So once in a while we will start with a location and rotation that is pretty close to one
        // that was already in the search list, so that will cost us one additional search, but later on
        // they will be merged anyway. So better to do the double search than have a blocking mechanism
        // APOX TODO: check not against recent position, but against list of previous locations
        if( fabs( pos.x - recentPos.x ) < 200.0f ) {nearByPoint = true;}
        if( fabs( pos.y - recentPos.y ) < 150.0f ) {nearByPoint = true;}
        if( nearByPoint && numTries > 20 ) {
            // prevent getting in an endless loop
            cout << "multiCam: Determine Position WARNING: it took to many tries to get a correct random position" << endl;
            nearByPoint = false;
        }
        numTries++;
#else
        if (recentPos.x > 100000) {
            printf("dummy to suppress compile warning about not used parameter\n");
        }
#endif
    } while (nearByPoint && useRecentPos); // we only need the nearby check if the location was not found in the previous frame

    return pos;
}

void determinePosition::pointsToPosition() {
    if (conf->getManualMode()) {
        if (locList.size() < 1) { // at least one location should be available for  manual mode
            detPosSt newPos;
            locList.push_back(newPos);
        }

        locList[0].pos = conf->getManual();
        // but we want to get the score results from this manual position
        scoreStruct tmp = threads[0].costTable->calcSimple(locList[0].pos, scoreThreshHold);
        locList[0].score = tmp.score;
        locList[0].amountOffFloor = tmp.amountOffFloor;
        locList[0].amountOnFloor = tmp.amountOnFloor;
        locList[0].numberOfTries = 0;
        locList[0].lastActive = 0; // need lower then threshold for visualization

        if (locList.size() > 1) { // remove locations that are created in non manual mode
            locList.erase(locList.begin(), locList.begin() + locList.size());
        }
    } else {
        // reduce the locList size to the available thread positions (numThreads)
        // subtract one more location, so we can add the random position
        if (locList.size() >= (numThreads - 1)) {
            locList.erase(locList.begin() + (numThreads - 1), locList.begin() + locList.size());
        }

        // create a new random position
        detPosSt randomLoc;
        positionStDbl zeroLoc;
        zeroLoc.x = 0.0f;
        zeroLoc.y = 0.0f;
        zeroLoc.rz = 0.0f;
        randomLoc.pos = createRandomPosition(zeroLoc, 0); // APOX todo, randomPosition should not be close to one of the found positions
        randomLoc.score = 1.0f; // this new random location always is thrown out, depending on the solver result, the new found position might be kept
        randomLoc.lastActive = 0;
        randomLoc.age = 0;
        locList.push_back(randomLoc);

        // now run localization algorithm on all previous found positions, including one new random location
        uint locSize = locList.size(); // use intermediate variable, because the size changes during the loop

        // run for each input location the optimization process
        void *ret;

        // double check if the amount of localization searches fits in the available threads
        if (locSize > numThreads) {
            cout << "multiCam: Determine Position Error: not enough threads to run all localization searches" << endl;
            exit(1);
        }

        // start the searches in different threads
        for (uint ii = 0; ii < locSize; ii++) {
            threads[ii].start = locList[ii];
            if (pthread_create(&(threads[ii].thread), NULL, processOneLocation, &(threads[ii].thread))) {
                cout << "WolrdSensing Determine Position Error creating threads" << endl;
                exit(0);
            }
        }

        // wait until all threads are finished
        for (uint ii = 0; ii < locSize; ii++) {
            pthread_join(threads[ii].thread, &ret);
            if (threads[ii].found.score < scoreThreshHold) {
                locList.push_back(threads[ii].found);
            }
            // update statistics old location, used in case derived position worse
            // if derived location better then this old location will be removed in the distance check
            locList[ii].lastActive++; // this location is not updated in this frame, increase last active moment
            locList[ii].age++; // location exist one frame longer
        }

        // check if positions are the same, if so, reject the one with the worst score (by setting score to 1.0f)
        for (uint ii = 0; ii < locList.size() - 1; ii++) {
            for (uint jj = ii + 1; jj < locList.size(); jj++) {
                // cout << "ii: " << ii << " jj: " << jj << endl;
                double x = locList[ii].pos.x - locList[jj].pos.x;
                double y = locList[ii].pos.y - locList[jj].pos.y;
                // TODO: add the rz also in this equation, because for the same location delta rz shall also be small
                double distancePow2 = x * x + y * y;

                if (distancePow2 < sameLocRangePow2) {
                    double iiScore = locList[ii].score + locList[ii].lastActive * 0.01f; // score also depends on when last found, older locations get a penalty, that will be significant after 20 frames
                    double jjScore = locList[jj].score + locList[jj].lastActive * 0.01f;
                    if (iiScore < jjScore) {
                        locList[jj].score = 1.0f; // ii better, reject jj
                    } else {
                        locList[ii].score = 1.0f; // jj better, reject ii
                    }
                    // use the oldest age, independent which had a better score
                    if (locList[ii].age < locList[jj].age) {
                        locList[ii].age = locList[jj].age;
                    } else {
                        locList[jj].age = locList[ii].age;
                    }
                    // cout << "same location: distance " << sqrt(distancePow2) << ", age " << locList[jj].age << ", loc " << locList[ii].pos.x << " " << locList[ii].pos.y << ", " << locList[jj].pos.x << " " << locList[jj].pos.y << endl;
                } else {
                    // because the field is symmetrical, also check if we are looking at the mirror point, if so, this can be treated as the same point
                    x = ((double) rFloor->getXFloorRight() - locList[ii].pos.x) - locList[jj].pos.x;
                    y = ((double) rFloor->getYFloorBottom() - locList[ii].pos.y) - locList[jj].pos.y;
                    // TODO: add the rz also in this equation, because for the same location delta rz shall also be small
                    distancePow2 = x * x + y * y;
                    if (distancePow2 < sameLocRangePow2) {
                        double iiScore = locList[ii].score + locList[ii].lastActive * 0.01f; // score also depends on when last found, older locations get a penalty, that will be significant after 20 frames
                        double jjScore = locList[jj].score + locList[jj].lastActive * 0.01f;
                        if (iiScore < jjScore) {
                            locList[jj].score = 1.0f; // ii better, reject jj

                            // preserve the oldest lock
                            if (locList[ii].age < locList[jj].age) {
                                locList[ii].age = locList[jj].age; // so jj was older but ii better, keep the same floor half of the oldest loc
                                // this means we have to mirror ii to the other side of the floor, also rz of ii has to be mirrored
                                locList[ii].pos.x = (double) rFloor->getXFloorRight() - locList[ii].pos.x;
                                locList[ii].pos.y = (double) rFloor->getYFloorBottom() - locList[ii].pos.y;
                                if (locList[ii].pos.rz > 180.0f) {
                                    locList[ii].pos.rz -= 180.0f;
                                } else {
                                    locList[ii].pos.rz += 180.0f;
                                }
                            }
                        } else {
                            locList[ii].score = 1.0f; // jj better, reject ii

                            // preserve the oldest lock
                            if (locList[ii].age > locList[jj].age) {
                                locList[jj].age = locList[ii].age; // so ii was older but jj better, keep the same floor half of the oldest loc
                                // this means we have to mirror jj to the other side of the floor, also rz of jj has to be mirrored
                                locList[jj].pos.x = (double) rFloor->getXFloorRight() - locList[jj].pos.x; // but we now use the mirror point
                                locList[jj].pos.y = (double) rFloor->getYFloorBottom() - locList[jj].pos.y;
                                if (locList[jj].pos.rz > 180.0f) {
                                    locList[jj].pos.rz -= 180.0f;
                                } else {
                                    locList[jj].pos.rz += 180.0f;
                                }
                            }
                        }
                    }
                    // cout << "same mirror location: distance " << sqrt(distancePow2) << ", age " << locList[jj].age << ", loc " << locList[ii].pos.x << " " << locList[ii].pos.y << ", " << locList[jj].pos.x << " " << locList[jj].pos.y << endl;
                }
            }
        }

        /*
         cout << "elements before " << locList.size() << "  ";
         for ( uint ii = 0; ii < locList.size() ; ii++ ) {
         cout << locList[ii].score << ", ";
         }
         cout << endl;
         */
        // remove locations that are not required anymore (in which case the score is set to 1.0f)
        // running reverse through the index, because i don't know what happens with the index to a higher element when a lower element was removed
        for (int ii = (int) locList.size() - 1; ii >= 0; ii--) { // use index type int because the check stops when negative
            if (locList[ii].score >= 1.0f) {
                // cout << "erase  ii " << ii << ", size " << locList.size();
                locList.erase(locList.begin() + ii);
                // cout << ", again size " << locList.size() << endl;
            }
        }

        // sort list on score
        std::sort(locList.begin(), locList.end());
        /*
         cout << "elements after  " << locList.size() << "  ";
         for ( uint ii = 0; ii < locList.size() ; ii++ ) {
         cout << locList[ii].score << " (" << locList[ii].lastActive << "), ";
         }
         cout << endl;
         */
    }
    exportMutex.lock();
    locListExport = locList;
    exportMutex.unlock();

    goodEnough();

    if (locList.size() > 0) {
        notifyNewPos();
        // Send new position to WorldModel if good enough
        // if not good enough, then for diagnostics purposes we still want to see
    }
}

vector<detPosSt> determinePosition::getLocList() {
    exportMutex.lock();
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
// when none of the new candidates is close then increase the lostLoc and do not provide a new localization candidate
// when the locLost exceeds a threshold switch to state 2 (locConfident == false)

// state 2 (locConfident == false)
// we have no clue where we are
// now search the candidate list for a NEW candidate with a very good score that exist for a while (age)
// if such a candidate is available, take that candidate, otherwise do not provide a new localization candidate

void determinePosition::goodEnough() {

    struct candidateSelectorSt {
        double error; // combined error of multiple variables e.g. xDelta, score, ..
        double xDelta; // TODO: remove next 3 variables
        double yDelta;
        double rzDelta;
        size_t index; // index that points to the candidate in the locList
        bool mirror; // use the position of the candidate or the mirrored position of the candidate
        bool operator<(const candidateSelectorSt& val) const {
            // sorting this struct is performed on error (lowest value first)
            return error < val.error;
        }
    };

    // create a list that contains indexes for locList with possible candidates for the localization
    vector<candidateSelectorSt> candSelList;

    if (locConfident) {
        // so we are pretty sure where we are, try to find the best candidate in the list of NEW list that is close to the previous candidate
        for (size_t ii = 0; ii < locList.size(); ii++) { // iterate through candidates
            if (locList[ii].lastActive == 0) { // only use when new
                for (size_t jj = 0; jj < 2; jj++) { // normal and mirrored location
                    candidateSelectorSt tmp;
                    double xDelta, yDelta, rzDelta;
                    tmp.index = ii;
                    if (jj == 0) {
                        xDelta = abs(goodEnoughLoc.pos.x - locList[ii].pos.x);
                        yDelta = abs(goodEnoughLoc.pos.y - locList[ii].pos.y);
                        rzDelta = fmod((goodEnoughLoc.pos.rz - locList[ii].pos.rz + 360.0), 360.0); // always positive (range 0:360)
                        tmp.mirror = false;
                    } else {
                        // mirror
                        xDelta = abs(goodEnoughLoc.pos.x - ((double) rFloor->getXFloorRight() - locList[ii].pos.x));
                        yDelta = abs(goodEnoughLoc.pos.y - ((double) rFloor->getYFloorBottom() - locList[ii].pos.y));
                        rzDelta = fmod((goodEnoughLoc.pos.rz - locList[ii].pos.rz + 360.0 + 180.0), 360.0); // always positive (range 0:360)
                        tmp.mirror = true;
                    }
                    if (rzDelta > 180) {
                        rzDelta = 360.0 - rzDelta;
                    } // an rz delta of e.g. 359 is the same as rz delta of 1 (360-359)
                    tmp.xDelta = xDelta;
                    tmp.yDelta = yDelta;
                    tmp.rzDelta = rzDelta;
                    // the next error only used to sort the canditates, the threshold is applied separate for x, y and rz
                    tmp.error = xDelta + yDelta + conf->getGoodEnough().angleToPixelRatio * rzDelta; // assume one rz degree displacement has the same error as pixel displacement
                    candSelList.push_back(tmp);
                }
            }
        }
    } else {
        // so we have no clue about the current location, try to find one
        // it should be new, exists for a while and have a good score
        for (size_t ii = 0; ii < locList.size(); ii++) {
            if ((locList[ii].lastActive == 0) && (locList[ii].age > conf->getGoodEnough().newMinAge)) { // only new candidates that exist for a few seconds
                // combine the score and age to a decision variable (lower is better)
                // which requires scaling of the age in the score range (which is in the order of 0.1)
                // (log(1000) - log(1)) / 10  = 0.69
                // (log(1000) - log(10)) / 10 = 0.46
                // (log(1000) - log(100)) / 10 = 0.23
                // (log(1000) - log(200)) / 10 = 0.16
                // (log(1000) - log(500)) / 10 = 0.069
                // (log(1000) - log(1000)) / 10 = 0
                // log returns natural logarithm (2.73 instead of e.g. log10)
                if (locList[ii].age < 1) {
                    // TODO: check if age is starting at 0, if so, increase by 1
                    // TODO: remove this check
                    printf("determinePosition ERROR: age is %d, which is < 1\n", locList[ii].age);
                }
                candidateSelectorSt tmp;
                tmp.index = ii;
                tmp.error = (log(1000) - log(locList[ii].age)) / 10.0; // a bad age results in > 0.2 a good age results in < 0.05
                if (tmp.error < 0) {
                    tmp.error = 0;
                } // score will go negative if living longer then 1000
                tmp.error += locList[ii].score; // combine age and score
                if (tmp.error > 1.0) {
                    tmp.error = 1.0;
                } // truncate in case a very short age and bad score
                candSelList.push_back(tmp);
            }
        }
    } // locConfident

    // now we have the indexes to the locList with possible candidates
    // sort this list (on the error), to get the best candidate
    sort(candSelList.begin(), candSelList.end());
#ifdef NONO
    for( size_t ii = 0; ii < candSelList.size(); ii++ ) {
        size_t locListIndex = candSelList[ii].index;
        printf("ii %2zu, index %2zu, last %3d, score %4.3f x %3.1f, y %3.1f, rz %3.1f, error %4.1f, mirror %d\n",
                ii, locListIndex, locList[locListIndex].lastActive, locList[locListIndex].score, candSelList[ii].xDelta,
                candSelList[ii].yDelta, candSelList[ii].rzDelta, candSelList[ii].error, candSelList[ii].mirror);
    }
    printf("\n");
#endif
    // TODO: verify if the sorting works correctly (smallest error first)
    // the first element in the list (lowest error) contains the index to the locList with the best candidate
    // printf("detPos num %zu conf %d", candSelList.size(), locConfident);

    // use this index to get the position from the locList and mirror variable to determine if this positions shall be mirrored
    // now check if the candidate is good enough (has good score and available for a while)
    // note: the error value has a different meaning for locConfident == true state and locConfident == false state
    exportMutex.lock();
    goodEnoughLoc.goodEnough = false; // default the location is not good enough
    if (lostLoc != INT_MAX) {
        lostLoc++; // default we do not have lock, and increase watchdog and make goodEnough.lastActive higher
    }
    if (locConfident) {
        // so we had a valid position in the recent past
        if (candSelList.size() >= 1) {
            // convert from meters to floorpixels
            double xDeltaThreshold = conf->getGoodEnough().xDeltaKeepThreshold * rFloor->getMetersToPixels();
            double yDeltaThreshold = conf->getGoodEnough().yDeltaKeepThreshold * rFloor->getMetersToPixels();
            double rzDeltaThreshold = conf->getGoodEnough().rzDeltaKeepThreshold;

            if ((candSelList[0].xDelta < xDeltaThreshold) && (candSelList[0].yDelta < yDeltaThreshold)
                    && (candSelList[0].rzDelta < rzDeltaThreshold)) {
                // so the new candidate is close to the previous known location
                lostLoc = 0; // reset the know where we are watchdog
                goodEnoughLoc = locList[candSelList[0].index];
                goodEnoughLoc.goodEnough = true;
                if (candSelList[0].mirror) {
                    // use the mirrored version of this location
                    goodEnoughLoc.pos.x = (double) rFloor->getXFloorRight() - goodEnoughLoc.pos.x;
                    goodEnoughLoc.pos.y = (double) rFloor->getYFloorBottom() - goodEnoughLoc.pos.y;
                    if (goodEnoughLoc.pos.rz > 180.0) {
                        goodEnoughLoc.pos.rz -= 180.0;
                    } else {
                        goodEnoughLoc.pos.rz += 180.0f;
                    }
                }
            } else {
                // printf(" x %3.1f < %3.1f, y %3.1f < %3.1f rz %3.1f < %3.1f\n", candSelList[0].xDelta, xDeltaThreshold, candSelList[0].yDelta, yDeltaThreshold, candSelList[0].rzDelta, rzDeltaThreshold);
            }
        }
        if (lostLoc > conf->getGoodEnough().keepWatchdog) { // 30fps * 4 = 4 seconds
            locConfident = false;
        }
    } else {
        // so we did not have a candidate in the past, lets checkout if the current one is good enough (if any)
        if (candSelList.size() >= 1) {
            // so there is a candidate
            // printf(" score %f, age %d", locList[candSelList[0].index].score, locList[candSelList[0].index].age);
            if (candSelList[0].error < conf->getGoodEnough().newThreshold) {
                // the candidate is good enough
                // use the "very old" position to select the candidate or mirrored candidate
                double xDelta = abs(goodEnoughLoc.pos.x - locList[candSelList[0].index].pos.x);
                double yDelta = abs(goodEnoughLoc.pos.y - locList[candSelList[0].index].pos.y);
                double rzDelta = fmod((goodEnoughLoc.pos.rz - locList[candSelList[0].index].pos.rz + 360.0), 360.0); // always positive (range 0:360)
                if (rzDelta > 180) {
                    rzDelta = 360.0 - rzDelta;
                } // an rz delta of e.g. 359 is the same as rz delta of 1 (360-359)
                double errorStraight = xDelta + yDelta + conf->getGoodEnough().angleToPixelRatio * rzDelta; // assume one rz degree has the same error as pixel
                // calculate the same for the mirrored position
                xDelta = abs(
                        goodEnoughLoc.pos.x
                                - ((double) rFloor->getXFloorRight() - locList[candSelList[0].index].pos.x));
                yDelta = abs(
                        goodEnoughLoc.pos.y
                                - ((double) rFloor->getYFloorBottom() - locList[candSelList[0].index].pos.y));
                rzDelta = fmod((goodEnoughLoc.pos.rz - locList[candSelList[0].index].pos.rz + 360.0 + 180.0), 360.0); // always positive (range 0:360)
                if (rzDelta > 180) {
                    rzDelta = 360.0 - rzDelta;
                } // an rz delta of e.g. 359 is the same as rz delta of 1 (360-359)
                double errorMirror = xDelta + yDelta + conf->getGoodEnough().angleToPixelRatio * rzDelta; // assume one rz degree displacement has the same error as pixel displacement

                // select the one that is closest to the "very old" known position
                if (errorMirror < errorStraight) {
                    candSelList[0].mirror = true;
                } else {
                    candSelList[0].mirror = false;
                }

                goodEnoughLoc = locList[candSelList[0].index];
                goodEnoughLoc.goodEnough = true;
                if (candSelList[0].mirror) {
                    // use the mirrored version of this location
                    goodEnoughLoc.pos.x = (double) rFloor->getXFloorRight() - goodEnoughLoc.pos.x;
                    goodEnoughLoc.pos.y = (double) rFloor->getYFloorBottom() - goodEnoughLoc.pos.y;
                    if (goodEnoughLoc.pos.rz > 180.0) {
                        goodEnoughLoc.pos.rz -= 180.0;
                    } else {
                        goodEnoughLoc.pos.rz += 180.0f;
                    }
                }
                // we know where we are, change to the other state
                locConfident = true; // go to other state where we know where we are
                lostLoc = 0; // reset the know where we are watchdog
            }
        }
    }
    goodEnoughLoc.lastActive = lostLoc;
    linePoint->setGoodEnoughLoc(goodEnoughLoc); // send last position to localization to filter the goal net for robot1
    exportMutex.unlock();
    // printf(" watch %4zu sort candidates:", lostLoc);
    // for( size_t ii = 0; ii < candSelList.size(); ii++ ) {
    //    printf(" %6.3f", candSelList[ii].error );
    // }
    // printf("\n");
}

detPosSt determinePosition::getGoodEnoughLoc() {
    exportMutex.lock();
    detPosSt retVal = goodEnoughLoc;
    exportMutex.unlock();
    return retVal;
}

detPosSt determinePosition::getGoodEnoughLocExport() {
    detPosSt retVal = getGoodEnoughLoc();
    // x should not be set outside the field
    if (retVal.pos.x < 0) {
        printf("ERROR     : localization x position of %.1f is out of field, it should be higher then 0.0\n",
                retVal.pos.x);
        retVal.pos.x = 0.0;
    }
    double xFieldEnd = (double) (rFloor->getWidth() - 1);
    if (retVal.pos.x > xFieldEnd) {
        printf("ERROR     : localization x position of %.1f is out of field, it should be lower then %.1f\n",
                retVal.pos.x, xFieldEnd);
        retVal.pos.x = xFieldEnd;
    }

    // y should not be set outside the field
    if (retVal.pos.y < 0) {
        printf("ERROR     : localization y position of %.1f is out of field, it should be higher then 0.0\n",
                retVal.pos.y);
        retVal.pos.y = 0.0;
    }
    double yFieldEnd = (double) (rFloor->getHeight() - 1);
    if (retVal.pos.y > yFieldEnd) {
        printf("ERROR     : localization y position of %.1f is out of field, it should be lower then %.1f\n",
                retVal.pos.y, yFieldEnd);
        retVal.pos.y = yFieldEnd;
    }
    // normalize to prevent issues when running of range for remote viewer export
    retVal.pos.rz = fmod(360.0 + retVal.pos.rz, 360.0); // in degrees
    return retVal;
}

detPosSt determinePosition::getGoodEnoughLocExportRos() {
    detPosSt retVal = getGoodEnoughLocExport();

    // calculate the real world x, y and rz (meters and radians)
    int xCenter = (rFloor->getWidth() - 1) / 2; // Width is odd number, -1 because index starts at 0
    int yCenter = (rFloor->getHeight() - 1) / 2; // Height is odd number, -1 because index starts at 0
    // printf("INFO      : xWidth %d xCenter %d yWidth %d yCenter %d\n", rFloor->getWidth(), xCenter, rFloor->getHeight(), yCenter);
    double x = retVal.pos.x;
    double y = retVal.pos.y;
    double rz = retVal.pos.rz;
    retVal.pos.x = -(yCenter - y) / rFloor->getMetersToPixels(); // range -(6+1) to (6+1) meter, center point is 0, negative number is on the top, positive number is on the bottom
    retVal.pos.y = (x - xCenter) / rFloor->getMetersToPixels(); // range -(9+1) to (9+1) meter, center point is 0, negative number is on left half floor, positive numbers right half floor
    retVal.pos.rz = fmod((rz - 90.0 + 360), 360.0) * (M_PI / 180.0); // input range 0 to 360 degrees, 0 is on short access (pointing upwards in rectangular viewer), anti clock wise
    return retVal;
}

// TODO: cleanup function below
void determinePosition::notifyNewPos() {
    detPosStRosVect positionROS = getFLoorLocationsRos();
#ifndef NOROS
    std::vector<robotLocationType> robotLocations;

    for (int i = 0; i < std::min((int) positionROS.numPositions, NUMPOSITION); i++) {
        if (positionROS.lastActive[i] == 0) {
            // only send new positions that just have been updated
            robotLocationType robotLoc;

            robotLoc.setX(positionROS.x[i]);
            robotLoc.setY(positionROS.y[i]);
            robotLoc.setTheta(positionROS.rz[i]);
            robotLoc.setConfidence(1.0 - positionROS.score[i]); // switch good-bad [0,1] values
            robotLoc.setAge(positionROS.age[i]);
            robotLoc.setLastActive(positionROS.lastActive[i]);
            robotLoc.setFPS(positionROS.fps[i]);
            robotLoc.setLinePoints(positionROS.linePoints[i]);

            robotLocations.push_back(robotLoc);
        }
    }

    for (vector<observer*>::const_iterator iter = vecObservers.begin(); iter != vecObservers.end(); ++iter) {
        if (*iter != NULL) {
            (*iter)->update_own_position(robotLocations, conf->getLocLatencyOffset());
        }
    }
#else
    // containment for compile warning about not used variables
    (void) positionROS;
#endif
}

void determinePosition::attach(observer *observer) {
    vecObservers.push_back(observer);
}

void determinePosition::detach(observer *observer) {
    vecObservers.erase(std::remove(vecObservers.begin(), vecObservers.end(), observer), vecObservers.end());
}

detPosStRosVect determinePosition::getFLoorLocationsRos() {

    detPosSt goodEoughLoc = getGoodEnoughLocExportRos();

    detPosStRosVect retVal;
    retVal.numPositions = 1;
    // in the future all possible location candidates can be send to ros but the WorldModel filtering
    // is not yet optimal, for the time being multiCam only sends one localization candidate if is good enough
    if (goodEnoughLoc.goodEnough) {
        // TODO: align about +x and +y direction and where the 0 axis is for rz (it currently feels slightly inconsistent)
        retVal.y[0] = goodEoughLoc.pos.y; // range -(9+1) to (9+1) meter, center point is 0, negative number is on left half floor, positive numbers right half floor
        retVal.x[0] = goodEoughLoc.pos.x; // range -(6+1) to (6+1) meter, center point is 0, negative number is on the top, positive number is on the bottom
        retVal.rz[0] = goodEoughLoc.pos.rz; // range 0 to 360 degrees, 0 is on short access (pointing upwards in rectangular viewer), anti clock wise
        retVal.score[0] = goodEoughLoc.score; // range 0 (good) to 1 (bad), roughly below 0.2 is probably location lock -- note: confidence=1.0-score
        retVal.age[0] = 1.0 * goodEoughLoc.age / prep->getFps(); // fps might be unreliable at startup
        retVal.lastActive[0] = 1.0 * goodEoughLoc.lastActive / prep->getFps(); // fps might be unreliable at startup
#ifndef NOROS
        TRACE("x=%6.2f y=%6.2f phi=%6.2f score=%4.2f", retVal.x[0], retVal.y[0], retVal.rz[0], retVal.score[0]);
#endif
    } else {
        retVal.y[0] = 0;
        retVal.x[0] = 0;
        retVal.rz[0] = 0;
        retVal.score[0] = 1.0;
        retVal.age[0] = -1;
        retVal.lastActive[0] = -1;
    }
    retVal.linePoints[0] = linePoint->getLinePointsPolar().size();
    retVal.fps[0] = prep->getFps();

    return retVal;
}
