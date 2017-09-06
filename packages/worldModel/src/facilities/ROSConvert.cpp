 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ROSConvert.cpp
 *
 *  Created on: Oct 8, 2016
 *      Author: Tim Kouters
 */

#include "int/facilities/ROSConvert.hpp"

#include "cDiagnosticsEvents.hpp"

cameraType convertCameraType(const rosMsgs::s_camera_type camType)
{
	try
	{
		cameraType retVal = cameraType::INVALID;

		switch(camType.camera_type)
		{
			case rosMsgs::s_camera_type::TYPE_FRONT:
			{
				retVal = cameraType::FRONTCAMERA;
				break;
			}

			case rosMsgs::s_camera_type::TYPE_OMNI:
			{
				retVal = cameraType::OMNIVISION;
				break;
			}

			default:
			{
				TRACE_ERROR("Received invalid camera type %d", camType.camera_type);
				break;
			}
		}

		return retVal;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

rosMsgs::s_camera_type convertCameraType(const cameraType camType)
{
	try
	{
		rosMsgs::s_camera_type retVal;

		switch(camType)
		{
			case cameraType::FRONTCAMERA:
			{
				retVal.camera_type = rosMsgs::s_camera_type::TYPE_FRONT;
				break;
			}

			case cameraType::OMNIVISION:
			{
				retVal.camera_type = rosMsgs::s_camera_type::TYPE_OMNI;
				break;
			}

			default:
			{
				TRACE_ERROR("Received invalid camera type %d", camType);
				break;
			}
		}

		return retVal;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

robotStatusType convertRobotStatus(const rosMsgs::s_robot_status statusType)
{
	try
	{
		robotStatusType retVal = robotStatusType::INVALID;

		switch(statusType.status_type)
		{
			case rosMsgs::s_robot_status::TYPE_INPLAY:
			{
				retVal = robotStatusType::INPLAY;
				break;
			}

			case rosMsgs::s_robot_status::TYPE_OUTOFPLAY:
			{
				retVal = robotStatusType::OUTOFPLAY;
				break;
			}

			default:
			{
				TRACE_ERROR("Received invalid robot status type %d", statusType.status_type);
				break;
			}
		}

		return retVal;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

rosMsgs::s_robot_status convertRobotStatus(const robotStatusType statusType)
{
	try
	{
		rosMsgs::s_robot_status retVal;

		switch(statusType)
		{
			case robotStatusType::INPLAY:
			{
				retVal.status_type = rosMsgs::s_robot_status::TYPE_INPLAY;
				break;
			}

			case robotStatusType::OUTOFPLAY:
			{
				retVal.status_type = rosMsgs::s_robot_status::TYPE_OUTOFPLAY;
				break;
			}

			default:
			{
				TRACE_ERROR("Received invalid robot status type %d", statusType);
				break;
			}
		}

		return retVal;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool convertIsClaimedByTeam(const rosMsgs::BallPossession claimType)
{
	try
	{
		bool retVal = false;

		switch(claimType.type)
		{
			case rosMsgs::BallPossession::TYPE_FIELD:
			{
				retVal = false;
				break;
			}

			case rosMsgs::BallPossession::TYPE_TEAMMEMBER:
			{
				retVal = true;
				break;
			}

			default:
			{
				TRACE_ERROR("Received invalid claim type %d", claimType.type);
				break;
			}
		}

		return retVal;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

rosMsgs::BallPossession convertBallPossession(ballPossessionClass_t possession)
{
	try
	{
		rosMsgs::BallPossession possessionROS;

		possessionROS.robotID = possession.getRobotID();

		if(possession.hasBallPossession())
		{
			possessionROS.type = rosMsgs::BallPossession::TYPE_TEAMMEMBER;
		}
		else
		{
		    if (possession.getBallCloseToObstacle())
		    {
    			possessionROS.type = rosMsgs::BallPossession::TYPE_OPPONENT;
		    }
		    else
		    {
    			possessionROS.type = rosMsgs::BallPossession::TYPE_FIELD;
		    }
		}

		return possessionROS;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

ballPossessionClass_t convertBallPossession(const rosMsgs::BallPossession ballPossessionROS, const float x, const float y)
{
	try
	{
		ballPossessionClass_t possession(ballPossessionROS.robotID);
		possession.claimBallByCamera();
		if(ballPossessionROS.type == rosMsgs::BallPossession::TYPE_TEAMMEMBER)
		{
			possession.claimBallByBallHandlers(x, y);
		}

		return possession;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
