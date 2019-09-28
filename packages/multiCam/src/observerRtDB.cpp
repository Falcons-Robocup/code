 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include "observerRtDB.hpp"

#include <stdexcept>

#include "vector2d.hpp"
#include "FalconsCommon.h"
#include "tracing.hpp"
#include "configurator.hpp"


using std::runtime_error;

observerRtDB::observerRtDB(const uint robotID, const bool cameraCorrectlyMounted, const float minimumLockTime)
{
    _myRobotId = getRobotNumber();

    initializeRtDB();

    _minimumLockTime = minimumLockTime;
    _visionBallPossession = false;

    localizationUniqueObjectIDIdx = 0;
    ballUniqueObjectIDIdx = 0;
    obstacleUniqueObjectIDIdx = 0;

    _cameraOffset = 0.0;

    if(!cameraCorrectlyMounted)
    {
        _cameraOffset = M_PI;
    }
}

observerRtDB::~observerRtDB()
{

}

void observerRtDB::initializeRtDB()
{
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    
    // start thread which communicates data to worldModel and pokes it
    _heartBeatThread = boost::thread(boost::bind(&observerRtDB::heartBeatLoop, this));
}

bool observerRtDB::heartBeatTick()
{
    mtx.lock();
    
    // report
    // minimize log pollution (to quickly see the real issues)
    // tprintf("bPoss=%d #loc=%d #ball=%d #obst=%d", (int)_visionBallPossession, (int)_robotLocCandidates.size(), (int)_ballCandidates.size(), (int)_obstacleCandidates.size());
    
    // dispatch all data to worldModel
    // get the following datamembers, put them into RTDB
    _rtdb->put(VIS_BALL_POSSESSION, &_visionBallPossession);
    _rtdb->put(LOCALIZATION_CANDIDATES, &_robotLocCandidates);
    _rtdb->put(OBSTACLE_CANDIDATES, &_obstacleCandidates);
    _rtdb->put(BALL_CANDIDATES, &_ballCandidates);
    _rtdb->put(MULTI_CAM_STATISTICS, &_multiCamStats); // TODO: data is provided at 1Hz, we should not put at >30Hz
    // execution architecture: worldModel triggers on new ball candidates
    
    // clear all
    _robotLocCandidates.clear();
    _obstacleCandidates.clear();
    _ballCandidates.clear();

    // final one will trigger worldModel execution
    mtx.unlock();
    return true; // always OK; false would stop the loop
}

void observerRtDB::heartBeatLoop()
{
    float frequency = 30.0;
    rtime::loop(frequency, boost::bind(&observerRtDB::heartBeatTick, this));
}

void observerRtDB::update_own_position(std::vector<robotLocationType> robotLocations, double timestampOffset)
{

    mtx.lock();

    for(auto it = robotLocations.begin(); it != robotLocations.end(); it++)
    {
        if (it->getAge() > _minimumLockTime)
        {
            float final_angle = project_angle_mpi_pi(it->getTheta() + _cameraOffset);

            robotLocalizationMeasurement robotLocCandidate;

            robotLocCandidate.identifier = uniqueObjectID(_myRobotId, localizationUniqueObjectIDIdx);
            robotLocCandidate.timestamp = rtime::now() - timestampOffset;
            robotLocCandidate.x = it->getX();
            robotLocCandidate.y = it->getY();
            robotLocCandidate.theta = final_angle;
            robotLocCandidate.confidence = it->getConfidence();

            _robotLocCandidates.push_back(robotLocCandidate);

            localizationUniqueObjectIDIdx++;

            if (localizationUniqueObjectIDIdx > MAX_UNIQUEOBJID)
            {
                localizationUniqueObjectIDIdx = 0;
            }
        }
    }

    mtx.unlock();

}

template <typename T1, typename T2>
bool insertMeasurementInSrv(T1 const &it, T2 &objCandidate, float cameraOffset, float elevation, std::string what, rtime timestamp, cameraEnum camType)
{
    float converted_angle = project_angle_0_2pi(it->getAngle() + M_PI_2 + cameraOffset + M_PI); // M_PI workaround for cameraOffset inconsistency w.r.t. localization
    float r = it->getRadius();
    TRACE("LATENCY %s angle=%6.2f, radius=%6.2f, elevation=%6.2f, confidence=%6.2f", what.c_str(), converted_angle, r, elevation, it->getConfidence());

    // sanity check
    if (it->getRadius() < 0.1)
    {
        TRACE_WARNING_TIMEOUT(10.0, "got too small multiCam %s radius (%6.2f)", what.c_str(), it->getRadius());
        return false;
    }

    // cutoff: for now we do not send floating balls
    float max_object_distance = 7.0;
    float h = CAMERA_HEIGHT;
    float max_elevation = atan2(-h, max_object_distance);
    if ((r > max_object_distance) || (elevation > max_elevation))
    {   // reject
        return false;
    }

    // radius up to here is w.r.t. robot center at z=0, and used as such in diagnostics
    // but for worldModel, it needs to be w.r.t. the actual camera
    // especially for closeby balls this is a significant factor, otherwise worldModel would interpret the ball too closeby
    // so here we must re-calculate radius to be w.r.t. camera
    r = sqrt(r*r + h*h);

    // fill in the data
    objCandidate.timestamp = timestamp;
    objCandidate.source = camType;
    objCandidate.confidence = it->getConfidence();
    objCandidate.azimuth = converted_angle;
    objCandidate.elevation = elevation;
    objCandidate.radius = r;
    objCandidate.cameraX = 0;
    objCandidate.cameraY = 0;
    objCandidate.cameraZ = h;
    objCandidate.cameraPhi = 0;
    objCandidate.color = objectColorEnum::UNKNOWN;
    return true;
}

void observerRtDB::update_own_ball_position(std::vector<ballPositionType> ballLocations, double timestampOffset)
{
    mtx.lock();

    rtime timestamp = rtime::now() - timestampOffset;

    for(auto it = ballLocations.begin(); it != ballLocations.end(); it++)
    {
        if(it->getBallType() == ballType)
        {
            ballMeasurement ballCandidate;
            // for now behave as OMNI since we filter high data upstream
            bool measurementApproved = insertMeasurementInSrv(it, ballCandidate, _cameraOffset, it->getElevation(), "ball", timestamp, cameraEnum::OMNIVISION);

            if (measurementApproved)
            {
                // Set ball identifier
                ballCandidate.identifier = uniqueObjectID(_myRobotId, ballUniqueObjectIDIdx);
                ballUniqueObjectIDIdx++;

                if (ballUniqueObjectIDIdx > MAX_UNIQUEOBJID)
                {
                    ballUniqueObjectIDIdx = 0;
                }

                _ballCandidates.push_back(ballCandidate);
            }
        }
    }

    mtx.unlock();
}

void observerRtDB::update_own_obstacle_position(std::vector<obstaclePositionType> obstacleLocations, double timestampOffset)
{
    mtx.lock();

    rtime timestamp = rtime::now() - timestampOffset;

    for(auto it = obstacleLocations.begin(); it != obstacleLocations.end(); it++)
    {
        TRACE("Found new obstacle at radius %f, angle %f", it->getRadius(), it->getAngle());

        // radius is calculated over ground
        // calculate elevation such that obstacle is at z=0
        float h = CAMERA_HEIGHT;
        float elevation = atan2(-h, it->getRadius());

        obstacleMeasurement obstacleCandidate;
        bool measurementApproved = insertMeasurementInSrv(it, obstacleCandidate, _cameraOffset, elevation, "obstacle", timestamp, cameraEnum::MULTICAMERA);

        if (measurementApproved)
        {
            // Set obstacle identifier
            obstacleCandidate.identifier = uniqueObjectID(_myRobotId, obstacleUniqueObjectIDIdx);
            obstacleUniqueObjectIDIdx++;

            if (obstacleUniqueObjectIDIdx > MAX_UNIQUEOBJID)
            {
                obstacleUniqueObjectIDIdx = 0;
            }


            _obstacleCandidates.push_back(obstacleCandidate);
        }
    }

    mtx.unlock();
}

void observerRtDB::update_own_ball_possession(const bool hasPossession)
{
    mtx.lock();

    if (hasPossession)
    {
        _visionBallPossession = true;
    }
    else
    {
        _visionBallPossession = false;
    }

    mtx.unlock();
}

void observerRtDB::update_multi_cam_statistics(multiCamStatistics const &multiCamStats)
{
    mtx.lock();
    _multiCamStats = multiCamStats;
    mtx.unlock();
}

