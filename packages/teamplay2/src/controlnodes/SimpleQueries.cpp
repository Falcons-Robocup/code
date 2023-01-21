// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/controlnodes/SimpleQueries.hpp"

#include "int/stores/BallStore.hpp"
#include "int/stores/ConfigurationStore.hpp"
#include "int/stores/FieldDimensionsStore.hpp"
#include "int/stores/GameStateStore.hpp"
#include "int/stores/RobotStore.hpp"
#include "int/stores/ObstacleStore.hpp"

#include "int/rules/RuleStimulatePassing.hpp"

#include "int/types/BehaviorTreeTypes.hpp"

#include "int/utilities/Timer.hpp"
#include "int/adapters/RTDBInputAdapter.hpp"

#include "tracing.hpp"

void registerSimpleQueryControlNode(BT::BehaviorTreeFactory& btFactory, const std::string& node_name, const std::function<bool()>& simplequery_functor)
{

    BT::NodeBuilder builder = [simplequery_functor, node_name](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<SimpleQueryControlNode>(name, config, simplequery_functor);
    };

    BT::TreeNodeManifest manifest = {BT::NodeType::CONTROL, node_name, BT::PortsList()};
    btFactory.registerBuilder(manifest, builder);
}


// register all simple queries
void registerNodes_SimpleQueries(BT::BehaviorTreeFactory& btFactory)
{
    ///////////////////
    // ACTIVE_ROBOTS //
    ///////////////////
    registerSimpleQueryControlNode(btFactory, "IsLowestActiveRobotID", std::bind(isLowestActiveRobotID));
    registerSimpleQueryControlNode(btFactory, "IsHighestActiveRobotID", std::bind(isHighestActiveRobotID));
    registerSimpleQueryControlNode(btFactory, "IsSecondHighestActiveRobotID", std::bind(isSecondHighestActiveRobotID));
    registerSimpleQueryControlNode(btFactory, "IsThirdHighestActiveRobotID", std::bind(isThirdHighestActiveRobotID));
    registerSimpleQueryControlNode(btFactory, "IsOnlyActiveRobotID", std::bind(isOnlyActiveRobotID));

    //////////
    // BALL //
    //////////
    registerSimpleQueryControlNode(btFactory, "IsBallLocationKnown", std::bind(isBallLocationKnown));
    registerSimpleQueryControlNode(btFactory, "IsWithin1mOfBall", std::bind(within1mOfBall));
    registerSimpleQueryControlNode(btFactory, "DoesTeamHaveBall", std::bind(doesTeamHaveBall));

    ///////////////
    // OWN_ROBOT //
    ///////////////
    registerSimpleQueryControlNode(btFactory, "DoesOwnRobotHaveBall", std::bind(doesOwnRobotHaveBall));
    registerSimpleQueryControlNode(btFactory, "IsPassApproachingRobot", std::bind(isPassApproachingRobot));
    registerSimpleQueryControlNode(btFactory, "IsBallApproachingRobot", std::bind(isBallApproachingRobot));

    ////////////////
    // TEAMMEMBER //
    ////////////////
    registerSimpleQueryControlNode(btFactory, "IsAssistantPresent", std::bind(isAssistantPresent));
    registerSimpleQueryControlNode(btFactory, "DoesAssistantHaveBall", std::bind(doesAssistantHaveBall));

    ///////////////////
    // SHOOT_BLOCKED //
    ///////////////////
    registerSimpleQueryControlNode(btFactory, "IsShortTurnToGoalBlockedByOpponent", std::bind(isShortTurnToGoalBlockedByOpponent));
    registerSimpleQueryControlNode(btFactory, "IsLongTurnToGoalBlockedByOpponent", std::bind(isLongTurnToGoalBlockedByOpponent));

    ///////////
    // RULES //
    ///////////
    registerSimpleQueryControlNode(btFactory, "IsValidNumberOfPassesGiven", std::bind(isValidNumberOfPassesGiven));

    ///////////
    // ????? //
    ///////////
    registerSimpleQueryControlNode(btFactory, "IsInScoringPosition", std::bind(isInScoringPosition));
}

BT::NodeStatus SimpleQueryControlNode::tick()
{
    const size_t children_count = children_nodes_.size();

    if (children_count != 2)
    {
        throw std::logic_error("SimpleQueryControlNode '" + name() + "' must have 2 children");
    }

    setStatus(BT::NodeStatus::RUNNING);

    BT::NodeStatus return_status;
    bool result = _simplequery_functor();
    if ( result )
    {
        // If the previous tick was taking the other path, haltChild() on the other path.
        // This ensures that any RUNNING actions get halt()'ed.
        if (result != _previousResult)
        {
            haltChild(1);
        }

        // Tick first child
        return_status = children_nodes_[0]->executeTick();

        // haltChild on SUCCESS/FAILURE, so that it will be re-ticked next time
        if (return_status != BT::NodeStatus::RUNNING)
        {
            haltChild(0);
        }
    }
    else
    {
        // If the previous tick was taking the other path, haltChild() on the other path.
        // This ensures that any RUNNING actions get halt()'ed.
        if (result != _previousResult)
        {
            haltChild(0);
        }

        // Tick second child
        return_status = children_nodes_[1]->executeTick();

        // haltChild on SUCCESS/FAILURE, so that it will be re-ticked next time
        if (return_status != BT::NodeStatus::RUNNING)
        {
            haltChild(1);
        }
    }

    std::ostringstream oss;
    oss << name() << " = ";
    result ? oss << "true" : oss << "false";
    oss << "\n";
    oss << "returning: " << behTreeReturnEnumMapping.right.at( nodeStatusMapping.right.at(return_status) );
    TRACE_FUNCTION(oss.str().c_str());

    _previousResult = result;
    
    return return_status;
}



///////////////////
// ACTIVE_ROBOTS //
///////////////////

bool isLowestActiveRobotID()
{
    auto own_robot_id = teamplay::RobotStore::getInstance().getOwnRobot().getNumber();
    auto all_robots = teamplay::RobotStore::getInstance().getAllRobots();

    auto robots_with_lower_id = std::count_if(all_robots.begin(), all_robots.end(),
                                              [&](const teamplay::Robot &it) { return it.getNumber() < own_robot_id; });

    return robots_with_lower_id == 0;
}

bool isHighestActiveRobotID()
{
    auto own_robot_id = teamplay::RobotStore::getInstance().getOwnRobot().getNumber();
    auto all_robots = teamplay::RobotStore::getInstance().getAllRobots();

    auto robots_with_higher_id = std::count_if(all_robots.begin(), all_robots.end(),
                                               [&](const teamplay::Robot &it) { return it.getNumber() > own_robot_id; });

    return robots_with_higher_id == 0;
}

bool isSecondHighestActiveRobotID()
{
    auto own_robot_id = teamplay::RobotStore::getInstance().getOwnRobot().getNumber();
    auto all_robots = teamplay::RobotStore::getInstance().getAllRobots();

    auto robots_with_higher_id = std::count_if(all_robots.begin(), all_robots.end(),
                                               [&](const teamplay::Robot &it) { return it.getNumber() > own_robot_id; });

    return robots_with_higher_id == 1;
}

bool isThirdHighestActiveRobotID()
{
    auto own_robot_id = teamplay::RobotStore::getInstance().getOwnRobot().getNumber();
    auto all_robots = teamplay::RobotStore::getInstance().getAllRobots();

    auto robots_with_higher_id = std::count_if(all_robots.begin(), all_robots.end(),
                                               [&](const teamplay::Robot &it) { return it.getNumber() > own_robot_id; });

    return robots_with_higher_id == 2;
}

bool isOnlyActiveRobotID()
{
    auto all_robots = teamplay::RobotStore::getInstance().getAllRobots();

    return all_robots.size() == 1;
}




//////////
// BALL //
//////////
bool isBallLocationKnown()
{
    bool retVal = teamplay::BallStore::getBall().isLocationKnown();

    // Trace result
    std::ostringstream oss;
    oss << __func__ << ": ";
    retVal ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    return retVal;
}

bool within1mOfBall()
{
    bool retVal = false;

    teamplay::Ball ball = teamplay::BallStore::getBall();

    if (ball.isLocationKnown())
    {
        auto myPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
        Point3D ballLocation = ball.getPosition();
        Vector2D delta = Vector2D(ballLocation.x, ballLocation.y) - myPos;
        if (delta.size() < 1.0)
        {
            retVal = true;
        }
    }

    // Trace result
    std::ostringstream oss;
    oss << __func__ << ": ";
    retVal ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    return retVal;
}

bool doesTeamHaveBall()
{
    const auto all_robots = teamplay::RobotStore::getInstance().getAllRobots();
    bool retVal = std::any_of(all_robots.begin(), all_robots.end(),
                       [](const teamplay::Robot r) { return r.hasBall(); });

    // Trace result
    std::ostringstream oss;
    oss << __func__ << ": ";
    retVal ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    return retVal;
}



///////////////
// OWN_ROBOT //
///////////////
bool doesOwnRobotHaveBall()
{
    bool retVal = teamplay::RobotStore::getInstance().getOwnRobot().hasBall();

    // Trace result
    std::ostringstream oss;
    oss << __func__ << ": ";
    retVal ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    return retVal;
}

bool isPassApproachingRobot()
{
    std::ostringstream oss;
    oss << __func__;
    TRACE_FUNCTION(oss.str().c_str());

    bool retVal = false;
    try 
    {
        // same as isBallApproachingRobot, but with extra conditions:
        // - pass intention must be given
        // - pass shot must have been performed
        // - Timer must not have elapsed

        // Remember the location where the pass is going towards
        bool closestRobotToPassTarget = false;

        // check the state, update Timer
        static teamplay::Timer lastPassIntention;
        static double timeout = 0.0;
        auto robotList = teamplay::RobotStore::getInstance().getAllRobotsExclOwnRobot();
        for(auto it = robotList.begin(); it != robotList.end(); it++)
        {
            auto intention = tpRTDBInputAdapter::getInstance().getIntention(it->getNumber());
            if (intention.action == actionTypeEnum::PASS)
            {
                lastPassIntention.reset();

                // Calculate the timeout used for staying in intercept.
                // This should happen only once based on the distance to the ball on last intention
                Point2D ballPos = teamplay::BallStore::getInstance().getBall().getLocation();
                Point2D robotPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
                auto distance = calc_distance(ballPos, robotPos);
                if (distance)
                {
                    // A scaling factor of 2.0 seems about right:
                    // A pass over  3m / 2.0 = pass intention valid for 1.5s
                    // A pass over 10m / 2.0 = pass intention valid for 5.0s
                    float scaleFactor = 2.0;
                    timeout = distance / scaleFactor;
                }
                else
                {
                    // If we cannot calculate, take static 5.0 s.
                    timeout = 5.0;
                }

                // Also check if this Robot is the closest Robot to the pass target
                std::vector<teamplay::Robot> robotsByDistance = teamplay::RobotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo( Point2D(intention.position.x, intention.position.y) );
                if (robotsByDistance.at(0).isOwnRobot())
                {
                    closestRobotToPassTarget = true;
                    TRACE("This Robot is the closest Robot to the Pass target");
                }
            }
        }

        teamplay::Ball ball = teamplay::BallStore::getBall();
        bool ballLost = !ball.isLocationKnown();
        if (ballLost)
        {
            TRACE("No Ball location known. Stay in place for intercept.");
        }

        bool teammateRecentPassIntention = lastPassIntention.hasStarted() && !lastPassIntention.hasElapsed(timeout);
        if (teammateRecentPassIntention)
        {
            // timers not yet expired -> check Ball movement
            // note: intention also includes (x,y) coordinates, but there is no need to use these

            // TODO: also override minimum Ball speed to a lower value? (Ball pass may be very weak)
            bool ballApproaching = isBallApproachingRobot();
            if (ballLost || ballApproaching || closestRobotToPassTarget)
            {
                retVal = true;
                TRACE("staying in place to intercept. ");
            }
            TRACE("")
                << "ballLost=" << std::to_string(ballLost) << "; "
                << "ballApproaching=" << std::to_string(ballApproaching) << "; "
                << "closestRobotToPassTarget=" << std::to_string(closestRobotToPassTarget);
        }
        else
        {
            TRACE("No recent Pass intention known.");
        }
    } catch (std::exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    // Trace result
    std::ostringstream oss2;
    oss2 << __func__ << ": ";
    retVal ? oss2 << "true" : oss2 << "false";
    TRACE("") << oss2.str().c_str();

    return retVal;
}

bool isBallApproachingRobot()
{
    std::ostringstream oss;
    oss << __func__;
    TRACE_FUNCTION(oss.str().c_str());

    bool retVal = false;
    try
    {
        auto captureRadius = teamplay::ConfigurationStore::getConfiguration().getInterceptBallCaptureRadius();
        auto minimumSpeed = teamplay::ConfigurationStore::getConfiguration().getInterceptBallMinimumSpeed();

        // get Robot position
        geometry::Pose2D currentPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
        geometry::Pose2D currentPose = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();

        // get Ball data and position
        teamplay::Ball ball = teamplay::BallStore::getBall();
        geometry::Pose2D ballPosition = geometry::Pose2D(ball.getPosition().x, ball.getPosition().y, ball.getPosition().z);
        geometry::Velocity2D ballVelocity = geometry::Velocity2D(ball.getVelocity().x, ball.getVelocity().y, ball.getVelocity().z);

        // determine relative speed
        geometry::Pose2D ballPositionTransformed = ballPosition;
        geometry::Pose2D ballPositionRCS = ballPositionTransformed.transformFCS2RCS(currentPose);
        geometry::Velocity2D ballVelocityTransformed = ballVelocity;
        geometry::Velocity2D ballVelocityRCS = ballVelocityTransformed.transformFCS2RCS(currentPose);
        float ballSpeed = ballVelocity.size();
        Vector2D ballSpeedVec2D(ball.getVelocity().x, ball.getVelocity().y);
        Vector2D ballPositionVec2D(ball.getPosition().x, ball.getPosition().y);

        bool ballIsMovingTowardsUs = (ballVelocityRCS.y < 0);
        bool ballMovingFastEnough = (ballSpeed > minimumSpeed);

        //// Compute flag: ballIntersectsCaptureRadius
        bool ballIntersectsCaptureRadius = false;
        // span a strafing line using RCS coordinates
        geometry::Pose2D leftPosRcs(-captureRadius, 0, 0);
        geometry::Pose2D rightPosRcs(captureRadius, 0, 0);

        // modify currentpos, as if already facing the Ball, so we can use transformation
        geometry::Pose2D transformedCurrentPos = currentPos;
        transformedCurrentPos.Rz = angle_between_two_points_0_2pi(transformedCurrentPos.x, transformedCurrentPos.y, ball.getPosition().x, ball.getPosition().y);
        geometry::Pose2D leftPosFcs = leftPosRcs.transformRCS2FCS(transformedCurrentPos);
        geometry::Pose2D rightPosFcs = rightPosRcs.transformRCS2FCS(transformedCurrentPos);

        // intersect
        Vector2D ballProjection = ballPositionVec2D + (10.0 * ballSpeedVec2D); // make vector long enough
        Vector2D intersectResult;
        //Position2D intersectPos;
        if (intersect(ballPosition, ballProjection, leftPosFcs, rightPosFcs, intersectResult))
        {
            if ((intersectResult - currentPos).size() < captureRadius)
            {
                //TRACE("intercept: ") << std::to_string(intersectResult.x) << ", " << std::to_string(intersectResult.y);
                //intersectPos.phi = transformedCurrentPos.Rz;
                //intersectPos.x = intersectResult.x;
                //intersectPos.y = intersectResult.y;
                ballIntersectsCaptureRadius = true;
            }
        }

        TRACE("captureRadius=") << std::to_string(captureRadius)
            << " ballIntersectsCaptureRadius=" << std::to_string(ballIntersectsCaptureRadius)
            << " ballIsMovingTowardsUs=" << std::to_string(ballIsMovingTowardsUs)
            << " ballMovingFastEnough=" << std::to_string(ballMovingFastEnough);
        retVal = (ballIntersectsCaptureRadius && ballIsMovingTowardsUs && ballMovingFastEnough);

    } catch (std::exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    // Trace result
    std::ostringstream oss2;
    oss2 << __func__ << ": ";
    retVal ? oss2 << "true" : oss2 << "false";
    TRACE("") << oss2.str().c_str();

    return retVal;
}



////////////////
// TEAMMEMBER //
////////////////

/*!
 * \brief Is my assistant present?
 *
 * ATTACKER_MAIN checks for ATTACKER_ASSIST and vice versa
 * DEFENDER_MAIN checks for DEFENDER_ASSIST and vice versa
 * All others return false.
 */
bool isAssistantPresent()
{
    bool retVal = teamplay::RobotStore::getInstance().getAssistantOfOwnRobot() != boost::none;

    // Trace result
    std::ostringstream oss;
    oss << __func__ << ": ";
    retVal ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    return retVal;
}

bool doesAssistantHaveBall()
{
    auto own_assistant = teamplay::RobotStore::getInstance().getAssistantOfOwnRobot();

    bool retVal = false;

    if (own_assistant)
    {
        retVal = own_assistant->hasBall();
    }
    else
    {
        retVal = false;
    }

    // Trace result
    std::ostringstream oss;
    oss << __func__ << ": ";
    retVal ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    return retVal;
}





///////////////////
// SHOOT_BLOCKED //
///////////////////

void isTurnToPOIBlockedByOpponent(const Point2D& poi, bool& shortTurnBlocked, bool& longTurnBlocked)
{
    geometry::Pose2D robotPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();

    // deltaRzToPOI is the delta Rz from the robotPos.Rz to face POI.
    double deltaRzToPOI = angle_between_two_points_0_2pi(robotPos.x, robotPos.y, poi.x, poi.y) - robotPos.Rz;
    deltaRzToPOI = project_angle_mpi_pi(deltaRzToPOI);
    TRACE("deltaRzToPOI: ") << std::to_string(deltaRzToPOI);

    auto obstacles = teamplay::ObstacleStore::getInstance().getAllObstacles();
    for (const auto& obst: obstacles)
    {
        // only consider obstacles within 1.5m
        if (obst.getDistanceTo(robotPos) < 1.5) // TODO -- make configurable
        {
            TRACE("obstacle at ") << obst.getLocation().str() << " is within 1.5 meter";

            // deltaRzToObst is the delta Rz from the robotPos.Rz to face the obstacle.
            double deltaRzToObst = angle_between_two_points_0_2pi(robotPos.x, robotPos.y, obst.getLocation().x, obst.getLocation().y) - robotPos.Rz;
            deltaRzToObst = project_angle_mpi_pi(deltaRzToObst);
            TRACE("deltaRzToObst: ") << std::to_string(deltaRzToObst);

            // 1: poi < obst < 0 --> short turn blocked by obstacle
            // 2: poi < 0 < obst --> long turn blocked by obstacle
            // 3: obst < poi < 0 --> long turn blocked by obstacle
            // 4: obst < 0 < poi --> long turn blocked by obstacle
            // 5: 0 < obst < poi --> short turn blocked by obstacle
            // 6: 0 < poi < obst --> long turn blocked by obstacle

            // 1
            if (deltaRzToPOI <= deltaRzToObst && deltaRzToObst <= 0.0)
            {
                shortTurnBlocked = true;
            }
            // 2
            if (deltaRzToPOI <= 0.0 && 0.0 <= deltaRzToObst)
            {
                longTurnBlocked = true;
            }
            // 3
            if (deltaRzToObst <= deltaRzToPOI && deltaRzToPOI <= 0.0)
            {
                longTurnBlocked = true;
            }
            // 4
            if (deltaRzToObst <= 0.0 && 0.0 <= deltaRzToPOI)
            {
                longTurnBlocked = true;
            }
            // 5
            if (0.0 <= deltaRzToObst && deltaRzToObst <= deltaRzToPOI)
            {
                shortTurnBlocked = true;
            }
            // 6
            if (0.0 <= deltaRzToPOI && deltaRzToPOI <= deltaRzToObst)
            {
                longTurnBlocked = true;
            }
        }
    }
}

bool isShortTurnToGoalBlockedByOpponent()
{
    bool retVal = false;
    try
    {
        Point2D poi = teamplay::FieldDimensionsStore::getInstance().getFieldDimensions().getLocation(teamplay::FieldPOI::OPP_GOALLINE_CENTER);
        bool shortTurnBlocked = false;
        bool longTurnBlocked = false;

        isTurnToPOIBlockedByOpponent(poi, shortTurnBlocked, longTurnBlocked);

        if (shortTurnBlocked)
        {
            retVal = true;
        }
    }
    catch (std::exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    // Trace result
    std::ostringstream oss;
    oss << __func__ << ": ";
    retVal ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    return retVal;
}

bool isLongTurnToGoalBlockedByOpponent()
{
    bool retVal = false;
    try
    {
        Point2D poi = teamplay::FieldDimensionsStore::getInstance().getFieldDimensions().getLocation(teamplay::FieldPOI::OPP_GOALLINE_CENTER);
        bool shortTurnBlocked = false;
        bool longTurnBlocked = false;

        isTurnToPOIBlockedByOpponent(poi, shortTurnBlocked, longTurnBlocked);

        if (longTurnBlocked)
        {
            retVal = true;
        }
    }
    catch (std::exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    // Trace result
    std::ostringstream oss;
    oss << __func__ << ": ";
    retVal ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    return retVal;
}



///////////
// RULES //
///////////
bool isValidNumberOfPassesGiven()
{
    bool retVal = teamplay::RuleStimulatePassing::getInstance().isRuleValid();

    // Trace result
    std::ostringstream oss;
    oss << __func__ << ": ";
    retVal ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    return retVal;
}



///////////
// ????? //
///////////
bool isInScoringPosition()
{
    bool retVal = false;

    auto own_location = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
    auto opp_side = teamplay::FieldDimensionsStore::getFieldDimensions().getArea( teamplay::FieldArea::OPP_SIDE );

    // if ownRobot in opponent side
    if ( opp_side.includesPosition( Position2D(own_location.x, own_location.y, own_location.Rz) ) )
    {
        auto goal_location = teamplay::FieldDimensionsStore::getFieldDimensions().getLocation(teamplay::FieldPOI::OPP_GOALLINE_CENTER);

        /*
            * angle_to_goal is the angle (in FCS) at the location of the Robot towards the center of the opponent goal.
            * e.g. if the Robot is located on the line perpendicular to the center of the goal, angle_to_goal equals 0.5*PI
            */
        auto angle_to_goal = angle_between_two_points_0_2pi(own_location.x, own_location.y, goal_location.x, goal_location.y);
        auto distance_to_goal = calc_distance(own_location, goal_location);

        retVal = (  (teamplay::ConfigurationStore::getConfiguration().getMinimumAngleToGoal() < angle_to_goal)
                    && (angle_to_goal < teamplay::ConfigurationStore::getConfiguration().getMaximumAngleToGoal())
                    && (teamplay::ConfigurationStore::getConfiguration().getMinimumDistanceToGoal() < distance_to_goal)
                    && (distance_to_goal < teamplay::ConfigurationStore::getConfiguration().getMaximumDistanceToGoal()));
    }

    // Trace result
    std::ostringstream oss;
    oss << __func__ << ": ";
    retVal ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    return retVal;
}
