 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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

#include <set>

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
bool isLowestActiveRobotID(const std::map<std::string, std::string> &params);
bool isHighestActiveRobotID(const std::map<std::string, std::string> &params);
bool isSecondHighestActiveRobotID(const std::map<std::string, std::string> &params);
bool isThirdHighestActiveRobotID(const std::map<std::string, std::string> &params);
bool isOnlyActiveRobotID(const std::map<std::string, std::string> &params);

bool isSetPiece(const std::map<std::string, std::string> &params);
bool isOwnSetPiece(const std::map<std::string, std::string> &params);
bool isPrepareSetPiece(const std::map<std::string, std::string> &params);
bool isKickoffSetPiece(const std::map<std::string, std::string> &params);
bool isDroppedBallSetPiece(const std::map<std::string, std::string> &params);
bool isPenaltySetPiece (const std::map<std::string, std::string> &params);
bool isGoalkickSetPiece (const std::map<std::string, std::string> &params);
bool isCornerSetPiece (const std::map<std::string, std::string> &params);
bool isFreekickSetPiece (const std::map<std::string, std::string> &params);
bool isThrowinSetPiece (const std::map<std::string, std::string> &params);
bool isParkingSetPiece (const std::map<std::string, std::string> &params);

bool within1mOfBall(const std::map<std::string, std::string> &params);
bool doesTeamHaveBall(const std::map<std::string, std::string> &params);
bool doesOwnRobotHaveBall(const std::map<std::string, std::string> &params);
bool isOwnRobotAtOpponentSide(const std::map<std::string, std::string> &params);
bool isBallAtSide(const std::map<std::string, std::string> &params);
bool isOwnRobotClosestToPOI(const std::map<std::string, std::string> &params);
bool isOwnRobotSecondClosestToPOI(const std::map<std::string, std::string> &params);
bool isOwnRobotFurthestFromPOI(const std::map<std::string, std::string> &params);
bool isOwnRobotSetpieceDefenderAssist(const std::map<std::string, std::string> &params);
bool isOwnRobotNearestToBall(const std::map<std::string, std::string> &params);
bool isOwnRobotNearestToLastKnownBallLocation(const std::map<std::string, std::string> &params);
bool isBallLocationKnown(const std::map<std::string, std::string> &params);

bool isShortTurnToGoalBlockedByOpponent(const std::map<std::string, std::string> &params);
bool isLongTurnToGoalBlockedByOpponent(const std::map<std::string, std::string> &params);

bool isValidNumberOfPassesGiven(const std::map<std::string, std::string> &params);

bool isAssistentPresent(const std::map<std::string, std::string> &params);
bool isPotentialOppAttackerPresent(const std::map<std::string, std::string> &params);

bool isBallApproachingRobot(const std::map<std::string, std::string> &params);
bool isPassApproachingRobot(const std::map<std::string, std::string> &params);
bool isOpponentHalfReachable(const std::map<std::string, std::string> &params);

bool isInScoringPosition(const std::map<std::string, std::string> &params);
bool isShotOnGoalBlocked(const std::map<std::string, std::string> &params);
bool isLobShotOnGoalBlocked(const std::map<std::string, std::string> &params);
bool isPassToClosestTeammemberBlocked(const std::map<std::string, std::string> &params);
bool isPassToClosestAttackerBlocked(const std::map<std::string, std::string> &params);
bool isPassToFurthestAttackerBlocked(const std::map<std::string, std::string> &params);
bool isPassToFurthestDefenderBlocked(const std::map<std::string, std::string> &params);
bool isTipInBlocked(const std::map<std::string, std::string> &params);

bool doesAssistantHaveBall(const std::map<std::string, std::string> &params);
bool allRobotsActive(const std::map<std::string, std::string> &params);

bool defendingStrategyOn(const std::map<std::string, std::string> &params);
bool multipleOpponentsOnOwnHalf(const std::map<std::string, std::string> &params);
bool isAnAttackerOnOppHalf(const std::map<std::string, std::string> &params);

/*
 * Worldstate functions only used is test scripts, but useful for future use
 */
bool isBallInOwnPenaltyArea(const std::map<std::string, std::string> &params);
bool isBallInOpponentPenaltyArea(const std::map<std::string, std::string> &params);
bool isPathToBallBlocked(const std::map<std::string, std::string> &params);

/*
 * Worldstate functions used by others, not specifically for trees
 */
bool isInMatch(const std::map<std::string, std::string> &params);
bool isOpponentGoalKeeperInLeftCorner(const std::map<std::string, std::string> &params);
bool isOpponentGoalKeeperInRightCorner(const std::map<std::string, std::string> &params);
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
    boost::optional<teamplay::robot> getClosestTeammember(bool includeGoalie) const;
    boost::optional<teamplay::robot> getClosestTeammemberToOpponentGoal(const std::set<treeEnum> &withRole) const;
    boost::optional<teamplay::robot> getClosestAttacker(boost::optional<teamplay::fieldArea> area = boost::none) const;
    boost::optional<teamplay::robot> getClosestAttackerToOpponentGoal() const;
    boost::optional<teamplay::robot> getClosestDefender() const;
    boost::optional<teamplay::robot> getClosestDefenderToOpponentGoal() const;
    boost::optional<teamplay::robot> getClosestTeammemberToLocationXY(
        Point2D location, bool includeOwnRobot, bool includeGoalie, const std::set<treeEnum> &withRole = {}) const;

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

    boost::optional<Position2D> getPos2DFromStr(const std::map<std::string, std::string> &parameters, std::string &param);
    boost::optional<Position2D> getPositionOfPOI(const std::string POI) const;


private:
    cWorldStateFunctions();
    cWorldStateFunctions(cWorldStateFunctions const&) = delete;
    cWorldStateFunctions(cWorldStateFunctions &&) = delete;
    cWorldStateFunctions operator=(cWorldStateFunctions const&) = delete;
    cWorldStateFunctions operator=(cWorldStateFunctions &&) = delete;
    ~cWorldStateFunctions() = default;

    robotNumber _robotID;

    float _fieldWidth, _fieldLength;
    float _positionMargin;

    std::map<robotNumber, treeEnum> _robotRoles;
};

#endif // CWORLDSTATEFUNCTIONS_HPP_
