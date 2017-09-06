 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include "observerRos.hpp"

#include <stdexcept>

#include "ros/ros.h"
#include "WorldModelNames.h"
#include "vector2d.hpp"
#include "FalconsCommon.h"
#include "timeConvert.hpp"
#include <cDiagnosticsEvents.hpp>
#include "configurator.hpp"


using std::runtime_error;

observerRos::observerRos(const uint robotID, const bool cameraCorrectlyMounted, const float minimumLockTime)
    : diagSender(diagnostics::DIAG_VISION, 0)
{
	initializeROS();
    srvObstPos.clear();

    this->robotID = robotID;
    _minimumLockTime = minimumLockTime;

    _prev_x = 0.0;
    _prev_y = 0.0;
    _prev_theta = 0.0;

    _cameraOffset = 0.0;

    if(!cameraCorrectlyMounted)
    {
    	_cameraOffset = M_PI;
    }
}

observerRos::~observerRos()
{

}

void observerRos::initializeROS()
{
	std::string own_loc_service_name = WorldModelInterface::s_set_own_location;
	std::string ball_loc_service_name =
			WorldModelInterface::s_set_own_ball_location;
	std::string obst_loc_service_name =
			WorldModelInterface::s_set_own_obstacle_location;
	std::string own_ball_possession_service_name =
			WorldModelInterface::s_set_own_camera_ball_possession;
	ros::service::waitForService(own_loc_service_name);
	ros::service::waitForService(ball_loc_service_name);
	ros::service::waitForService(obst_loc_service_name);
	ros::service::waitForService(own_ball_possession_service_name);
	cPosition = hPosition.serviceClient<worldModel::set_own_location>(
			own_loc_service_name, true);
	cBallPosition = hPosition.serviceClient<worldModel::set_own_ball_location>(
			ball_loc_service_name, true);
	cObstaclePosition = hPosition.serviceClient<
			worldModel::set_own_obstacle_location>(obst_loc_service_name, true);
	cBallPossession = hPosition.serviceClient<
			worldModel::set_own_camera_ball_possession>(
			own_ball_possession_service_name, true);
}

void observerRos::update_own_position(std::vector<robotLocationType> robotLocations, double timestampOffset)
{

    mtx.lock();

    srvRobotPos.request.x.clear();
	srvRobotPos.request.y.clear();
	srvRobotPos.request.theta.clear();
	srvRobotPos.request.confidence.clear();
	srvRobotPos.request.timestamp.clear();

    for(auto it = robotLocations.begin(); it != robotLocations.end(); it++)
    {
		if (it->getAge() > _minimumLockTime)
		{
			float final_angle = project_angle_mpi_pi(it->getTheta() + _cameraOffset);
			srvRobotPos.request.x.push_back(it->getX());
			srvRobotPos.request.y.push_back(it->getY());
			srvRobotPos.request.theta.push_back(final_angle);
			srvRobotPos.request.confidence.push_back(it->getConfidence());
			srvRobotPos.request.timestamp.push_back(getTimeNow() - timestampOffset);
			// do not send age and lastActive, these were once interesting for diagnostics
		}
    }
    TRACE("sending %d position candidates to worldModel", robotLocations.size());

    //Send the message to the WorldModel node
	if (!cPosition.call(srvRobotPos))
	{
		TRACE_ERROR("Failed to update own position");
		initializeROS();
	}
    mtx.unlock();

    if(robotLocations.size() > 0)
    {
		mtx.lock();
		diagData.fps = robotLocations.at(0).getFPS();
		diagData.linePoints = robotLocations.at(0).getLinePoints();
		mtx.unlock();
    }

}

template <typename T1, typename T2>
void insertMeasurementInSrv(T1 const &it, T2 &srv, float cameraOffset, float z, std::string what, double timestamp, rosMsgs::s_camera_type camType)
{
    float converted_angle = project_angle_0_2pi(it->getAngle() + M_PI_2 + cameraOffset + M_PI); // M_PI workaround for cameraOffset inconsistency w.r.t. localization
    TRACE("LATENCY %s angle=%6.2f, radius=%6.2f, z=%6.2f, confidence=%6.2f", what.c_str(), converted_angle, it->getRadius(), z, it->getConfidence());
    
    // sanity check
    if (it->getRadius() < 0.1)
    {
        TRACE_ERROR("got too small omnivision %s radius (%6.2f)", what.c_str(), it->getRadius());
        return;
    }

    // calculate position (x,y,z) in RCS
    Vector3D posRcs(it->getRadius() * cos(converted_angle), it->getRadius() * sin(converted_angle), z);
    TRACE("%sRcs=(%6.2f, %6.2f)", what.c_str(), posRcs.x, posRcs.y);
    // calculate virtual camera position: assume omnivision mirror is at (0,0,h), 
    // then project behind it with a distance d
    float h = 0.80; // [m] omnivision mirror height
    float d = 0.00; // [m] distance from camera to mirror
    // values (h=0.80, d=0.00) decided by discussion JFEI/APOX 20161124
    Vector3D mirrpos(0, 0, h);
    Vector3D mirr2object = posRcs - mirrpos;
    Vector3D tmp = mirr2object; // copy needed because Normalize modifies the data, we need it unchanged below
    Vector3D cam2mirr = tmp.Normalize() * d;
    Vector3D campos = mirrpos - cam2mirr;
    
    // calculate spherical coordinates w.r.t. virtual camera position
    float az = 0.0; // by definition, because it is factored in robot cameraPhi
    float r = vectorsize(mirr2object) + d;
    float el = atan2(-h, it->getRadius()); // see sanity check above: radius may not be small
    
    // fill in the service data
    srv.request.azimuth.push_back(az);
    srv.request.elevation.push_back(el);
    srv.request.radius.push_back(r);
    srv.request.confidence.push_back(it->getConfidence());
    srv.request.timestamp.push_back(timestamp);
    srv.request.cameraType.push_back(camType);
    srv.request.relativeCameraX.push_back(campos.x);
    srv.request.relativeCameraY.push_back(campos.y);
    srv.request.relativeCameraZ.push_back(campos.z);
    srv.request.relativeCameraPhi.push_back(converted_angle);
    TRACE("cam: (x=%6.2f, y=%6.2f, z=%6.2f, phi=%6.2f)", campos.x, campos.y, campos.z, converted_angle);
}

void observerRos::update_own_ball_position(std::vector<ballPositionType> ballLocations, double timestampOffset)
{
    mtx.lock();

    // initialize
    srvBallPos = worldModel::set_own_ball_location();
    double timestamp = getTimeNow() - timestampOffset;
    rosMsgs::s_camera_type camType;
    camType.camera_type = rosMsgs::s_camera_type::TYPE_OMNI;

    for(auto it = ballLocations.begin(); it != ballLocations.end(); it++)
    {
    	if(it->getBallType() == ballType)
    	{
    		insertMeasurementInSrv(it, srvBallPos, _cameraOffset, it->getZ(), "ball", timestamp, camType);
    	}
    }
    
    // diagnostics
    diagData.numBalls = (int)ballLocations.size();

    //Send the message to the WorldModel node
    if (!cBallPosition.call(srvBallPos))
    {
        TRACE_ERROR("Failed to update ball position");
        initializeROS();
    }

    mtx.unlock();
}

void observerRos::update_own_obstacle_position(std::vector<obstaclePositionType> obstacleLocations, double timestampOffset)
{
    mtx.lock();

    worldModel::set_own_obstacle_location locObst;
    double timestamp = getTimeNow() - timestampOffset;
    rosMsgs::s_camera_type camType;
    camType.camera_type = rosMsgs::s_camera_type::TYPE_OMNI;

    for(auto it = obstacleLocations.begin(); it != obstacleLocations.end(); it++)
    {
    	TRACE("Found new obstacle at radius %f, angle %f", it->getRadius(), it->getAngle());

        insertMeasurementInSrv(it, locObst, _cameraOffset, 0.0, "obstacle", timestamp, camType);
    }

    // diagnostics
    diagData.numObstacles = (int)obstacleLocations.size();

    if (!cObstaclePosition.call(locObst))
	{
		TRACE_ERROR("Failed to update obstacle position");
		initializeROS();
	}
	
    mtx.unlock();
}

void observerRos::update_own_ball_possession(const bool hasPossession)
{
	mtx.lock();

	worldModel::set_own_camera_ball_possession ballPossession;
	ballPossession.request.ballInfrontOfRobot = hasPossession;

	if(!cBallPossession.call(ballPossession))
	{
		TRACE_ERROR("Failed to update visual ball possession");
		initializeROS();
	}

	mtx.unlock();
}

void observerRos::sendDiagnostics()
{
    mtx.lock();

    if (true) // diagnostics data is ALWAYS to be communicated
    {
        diagSender.set(diagData);
        // clear the diagnostics obstacle list
        diagData.obstacles.clear();
        // clear the diagnostics ball list
        diagData.ballpos.clear();    
        diagData.numBalls = 0;
        diagData.numObstacles = 0;
    }

    mtx.unlock();
}

bool observerRos::isSamePosition(const float x, const float y, const float theta)
{
    bool isSame = true;

    if((fabs(x - _prev_x) > 0.001) ||
       (fabs(y - _prev_y) > 0.001) ||
       (fabs(theta - _prev_theta) > 0.001))
    {
        isSame = false;
        _prev_x = x;
		_prev_y = y;
		_prev_theta = theta;
    }

    return isSame;
}
