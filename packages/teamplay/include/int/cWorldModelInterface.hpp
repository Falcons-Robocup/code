 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModelInterface.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef CWORLDMODELINTERFACE_HPP_
#define CWORLDMODELINTERFACE_HPP_

#include "int/types/cPositionTypes.hpp"
#include "int/types/cBallLocationTypes.hpp"
#include "int/types/cBallPossessionTypes.hpp"
#include "int/types/cRobotLocationTypes.hpp"

#include "position2d.hpp"
#include "vector3d.hpp"

#include <vector>

class cWorldModelInterface
{
	public:
			static cWorldModelInterface& getInstance()
			{
				static cWorldModelInterface instance; // Guaranteed to be destroyed.
											          // Instantiated on first use.
				return instance;
			}

			void updateWorldModelValues();

			/*
			 * Getter functionality for internal use (within Teamplay)
			 */
			bool getOwnLocation(Position2D &robotPosition);
			bool getOwnVelocity(Velocity2D &robotVelocity);
			void getTeammembers(robotLocations &teammembers);
			void getOpponents(robotLocations &opponents);
			void getBallPossession(ballPossession_struct_t &ballPossession);
			void getActiveRobots(std::vector<robotNumber> &activeRobots);

			/*
			 * Setter functionality for external use 
 			 * (cRosAdapterWorldModel will call these setters at a fixed frequency)
			 */
			void setOwnRobot(const Position2D&, const Velocity2D&); /* New-style */
			void setOwnLocation(Position2D const &robotPosition);   /* Old-style */
			void setOwnVelocity(Velocity2D const &robotVelocity);   /* Old-style */
			void setTeammembers(robotLocations const &teammembers, std::vector<robotNumber> const &activeRobots);
			void setOpponents(robotLocations const &opponents);
			void setBallLocation(ballLocations const &balls, ballLocation const &lastKnownBallLocation);
			void setBallPossession(ballPossession_struct_t const &ballPossession);
			void setBallClaimedLocation(Point3D const &claimLocation);

		private:
			    const size_t _NR_ROBOTS = 10;
			    robotLocations _robotTeammembers;
			    robotLocations _robotObstacles;
				Position2D _ownRobotPosition;
				Velocity2D _ownRobotVelocity;
				bool _ownRobotPositionValid;
				bool _ownRobotVelocityValid;
				ballPossession_struct_t _ballPossession;
				Point3D _claimedLocation;
				ballLocation _lastBallLocation;
				std::vector<robotNumber> _activeRobots;

				cWorldModelInterface();
				~cWorldModelInterface();
				cWorldModelInterface(cWorldModelInterface const&); // Don't Implement
				void operator=(cWorldModelInterface const&);	   // Don't implement
};

#endif /* CWORLDMODELINTERFACE_HPP_ */
