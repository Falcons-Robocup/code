 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldStateFunctions.hpp
 *
 *  Created on: Sep 18, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef CWORLDSTATEFUNCTIONS_HPP_
#define CWORLDSTATEFUNCTIONS_HPP_

#include <boost/optional.hpp>

#include "int/types/cBallLocationTypes.hpp"
#include "int/types/cDecisionTreeTypes.hpp"
#include "int/types/cPositionTypes.hpp"
#include "int/types/cRobotLocationTypes.hpp"
#include "int/types/robot.hpp"

#include "cEnvironmentField.hpp"


/*
 * WorldState functions used in trees
 */
bool isLowestActiveRobotID();
bool isHighestActiveRobotID();
bool isSecondHighestActiveRobotID();
bool isThirdHighestActiveRobotID();
bool isOnlyActiveRobotID();

bool returnTrue();
bool returnFalse();

bool isInMatch();

bool isSetPiece();
bool isOwnSetPiece();
bool isPrepareSetPiece();
bool isKickoffSetPiece();
bool isDroppedBallSetPiece();
bool isSidelineSetPiece();
bool isSidelineSetPieceRight();
bool isPenaltySetPiece ();
bool isGoalkickSetPiece ();
bool isCornerSetPiece ();
bool isFreekickSetPiece ();
bool isThrowinSetPiece ();

bool within1mOfBall();
bool doesTeamHaveBall();
bool doesOpponentHaveBall();
bool doesNoTeamHaveBall();
bool doesOwnRobotHaveBall();
bool isOwnRobotAtOpponentSide();
bool isBallAtOpponentSide();
bool isBallAtOwnSide();
bool isBallAtLeftSide();
bool isBallAtRightSide();
bool isOwnRobotNearestToBall();
bool isOwnRobotNearestToLastKnownBallLocation();
bool isBallLocationKnown();
bool isBallInOwnPenaltyArea();
bool isBallInOpponentPenaltyArea();

bool isMemberInOwnPenaltyArea();
bool isMemberInOpponentPenaltyArea();

bool isShortTurnToGoalBlockedByOpponent();
bool isLongTurnToGoalBlockedByOpponent();

bool isShotAtGoalAllowed();

bool ballPickupOnOpponentHalf();

bool isAssistentPresent();
bool isClosestAttackerToBall();
bool isClosestDefenderToBall();
bool isPotentialOppAttackerPresent();

bool isOpponentGoalKeeperInLeftCorner();
bool isOpponentGoalKeeperInRightCorner();

bool isBallApproachingRobot();
bool isOpponentHalfReachable();

bool isShotOnGoalBlocked();
bool isLobShotOnGoalBlocked();
bool isPassToClosestTeammemberBlocked();
bool isPassToClosestAttackerBlocked();
bool isPassToFurthestAttackerBlocked();
bool isTipInBlocked();
bool isPathToBallBlocked();

bool doesAssistantHaveBall();
bool allRobotsActive();

bool defendingStrategyOn();

bool multipleOpponentsOnOwnHalf();
bool isOpponentWithinXMeterFromOwnGoal();

/*
 * Worldstate functions to be used by others, not specifically for trees
 */
teamplay::robot getRobotClosestToPoint(const teamplay::robots& robots, const Point2D& point);


class cWorldStateFunctions
{
	public:
		static cWorldStateFunctions& getInstance()
		{
			static cWorldStateFunctions instance; // Guaranteed to be destroyed.
												  // Instantiated on first use.
			return instance;
		}

		// ball related
		boost::optional<float> ballDistance(const robotNumber &robotID);

		// team related
		bool isMemberInArea(areaName area, bool includeOwnRobot, bool includeGoalie);
		bool getClosestTeammember(double &target_x, double &target_y, bool includeGoalie);
		void getClosestAttacker(areaName area, double &target_x, double &target_y);
		void getClosestAttackerToOpponentGoal(double &target_x, double &target_y);
		void getClosestDefender(double &target_x, double &target_y);
		void getClosestMemberToLocationXY(double location_x, double location_y, bool includeOwnRobot, bool includeGoalie, bool &foundMember, uint8_t &robotID);

		// opponent related
		bool getClosestOpponent(float &target_x, float &target_y);
		bool getPotentialOpponentAttacker(float &target_x, float &target_y);
		bool getClosestOpponentToLocationXY(float &target_x, float &target_y, float location_x, float location_y);
		bool getSecondClosestOpponentToLocationXY(float &target_x, float &target_y, float location_x, float location_y);
		void getObstructingObstaclesInPath(const Point2D robotPos, const Point2D targetPos, const float radiusObjectMeters, std::vector<robotLocation> &obstacles);
		bool getPreferredShootXYOfGoal(float &target_x, float &target_y);

		Position2D getLocationOfRobot(const robotNumber &robotID);

		robotNumber getRobotID();
		void setRobotID(const robotNumber &robotID);

		void setRobotRole(const robotNumber &robotID, const treeEnum &role);
		treeEnum getRobotRole(const robotNumber &robotID);

		boost::optional<robotNumber> getRobotWithRole(const treeEnum &role);


	private:
		robotNumber _robotID;

		~cWorldStateFunctions();
		cWorldStateFunctions();
		cWorldStateFunctions(cWorldStateFunctions const&); // Don't Implement
		void operator=(cWorldStateFunctions const&);	   // Don't implement

		float _fieldWidth, _fieldLength;
		float _positionMargin;

		std::map<robotNumber, treeEnum> _robotRoles;
};

#endif /* CWORLDSTATEFUNCTIONS_HPP_ */
