// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <iterator>

#include "int/controlnodes/AdvancedQueries.hpp"

#include "int/stores/ConfigurationStore.hpp"
#include "int/stores/GameStateStore.hpp"

#include "int/WorldStateFunctions.hpp"

using namespace teamplay;



void registerNodes_AdvancedQueries(BT::BehaviorTreeFactory& btFactory)
{
    btFactory.registerNodeType<IsOwnRobotClosestToPOI>("IsOwnRobotClosestToPOI");
    btFactory.registerNodeType<IsPOIInArea>("IsPOIInArea");
    btFactory.registerNodeType<IsRole>("IsRole");
    btFactory.registerNodeType<IsSetpiece>("IsSetpiece");
    btFactory.registerNodeType<IsShootToPOIBlocked>("IsShootToPOIBlocked");
}

////////////////////////////
// IsOwnRobotClosestToPOI //
////////////////////////////

// Must have 2 or 4 children.
// If 2 children, it is considered True/False.
// If 4 children: First child is traversed if we are the closest robot, second child if we are the second closest robot, etc.
BT::NodeStatus IsOwnRobotClosestToPOI::tick()
{
    TRACE_FUNCTION("");
    const size_t children_count = children_nodes_.size();

    if (children_count != 2 && children_count != 4)
    {
        throw std::logic_error("IsOwnRobotClosestToPOI must have 2 or 4 children");
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Parse poi
    auto res = getInput<Point2D>("poi");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [poi]:", res.error());
    }
    Point2D poi = res.value();

    int indexOfMyRobotNumberSortedByDistance = 0;

    auto myRobotNumber = teamplay::RobotStore::getInstance().getOwnRobot().getNumber();
    auto sortedListOfRobots = teamplay::RobotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(poi);
    auto it = std::find(sortedListOfRobots.begin(), sortedListOfRobots.end(), myRobotNumber);

    if (it != sortedListOfRobots.end())
    {
        indexOfMyRobotNumberSortedByDistance = std::distance(sortedListOfRobots.begin(), it);
    }
    else
    {
        throw std::logic_error("IsOwnRobotClosestToPOI - myRobotNumber not found");
    }

    BT::NodeStatus return_status = BT::NodeStatus::FAILURE;
    if (children_count == 2)
    {
        // True if my robot number is the *first* (idx==0) in the list of robots sorted by distance to poi
        // i.e., my robot number is closest to poi
        bool result = (indexOfMyRobotNumberSortedByDistance == 0);
        if (result)
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

        // Trace result
        std::ostringstream oss;
        oss << "IsOwnRobotClosestTo " << poi << ": ";
        result ? oss << "true" : oss << "false";
        TRACE_FUNCTION(oss.str().c_str());

        _previousResult = result;

    }
    else if (children_count == 4)
    {
        // Tick the Nth child
        return_status = children_nodes_[indexOfMyRobotNumberSortedByDistance]->executeTick();

        // haltChild on SUCCESS/FAILURE, so that it will be re-ticked next time
        if (return_status != BT::NodeStatus::RUNNING)
        {
            haltChild(indexOfMyRobotNumberSortedByDistance);
        }
    }
    
    return return_status;
}

/////////////////
// IsPOIInArea //
/////////////////

BT::NodeStatus IsPOIInArea::tick()
{
    const size_t children_count = children_nodes_.size();

    if (children_count != 2)
    {
        throw std::logic_error("IsPOIInArea must have 2 children");
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Parse poi
    auto res = getInput<Point2D>("poi");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [poi]:", res.error());
    }
    Point2D poi = res.value();

    // Parse area
    auto res2 = getInput<Area2D>("area");
    if (!res2)
    {
        throw BT::RuntimeError("error reading port [area]:", res2.error());
    }
    Area2D area = res2.value();

    bool result = ( area.includesPosition( Position2D(poi.x, poi.y, 0.0) ) );

    // Trace result
    std::ostringstream oss;
    oss << "Is " << poi << " in Area: ";
    result ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    BT::NodeStatus return_status;
    if ( area.includesPosition( Position2D(poi.x, poi.y, 0.0) ) )
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

    _previousResult = result;
    
    return return_status;
}

////////////
// IsRole //
////////////

BT::NodeStatus IsRole::tick()
{
    const size_t children_count = children_nodes_.size();

    if (children_count != 2)
    {
        throw std::logic_error("IsRole must have 2 children");
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Parse role
    auto res = getInput<teamplay::RoleEnum>("role");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [role]:", res.error());
    }
    teamplay::RoleEnum role = res.value();


    bool result = ( teamplay::RobotStore::getInstance().getOwnRobot().getRole() == role ); 

    // Trace result
    std::ostringstream oss;
    oss << "Is role " << Role(role).str() << ": ";
    result ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    BT::NodeStatus return_status;
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

    _previousResult = result;
    
    return return_status;
}

////////////////
// IsSetpiece //
////////////////

bool isSetpiece(SetpieceTreeTypeEnum setpiece)
{
    bool returnVal = false;
    switch( setpiece )
    {
        case SetpieceTreeTypeEnum::ANY:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isSetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::OWN:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isOwnSetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::PREPARE:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isPrepareSetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::KICKOFF:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isKickoffSetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::CORNER:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isCornerSetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::THROWIN:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isThrowinSetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::GOALKICK:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isGoalkickSetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::FREEKICK:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isFreekickSetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::DROPPED_BALL:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isDroppedBallSetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::PENALTY:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isPenaltySetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::PARK:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isParkingSetPiece();
            break;
        }
        case SetpieceTreeTypeEnum::SHOWING_ALIVE:
        {
            returnVal = teamplay::GameStateStore::getInstance().getGameState().isShowingAliveSetPiece();
            break;
        }
        default:
        {
            throw std::runtime_error("Unknown SetpieceTreeTypeEnum");
        }
    }

    return returnVal;
}

BT::NodeStatus IsSetpiece::tick()
{
    const size_t children_count = children_nodes_.size();

    if (children_count != 2)
    {
        throw std::logic_error("IsSetpiece must have 2 children");
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Parse setpiece
    auto res = getInput<SetpieceTreeTypeEnum>("setpiece");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [setpiece]:", res.error());
    }
    SetpieceTreeTypeEnum setpiece = res.value();

    // assert query
    bool result = isSetpiece(setpiece);

    std::ostringstream oss;
    oss << setpieceTreeTypeEnumMapping.right.at(setpiece) << ": ";
    result ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    BT::NodeStatus return_status;
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

    _previousResult = result;
    
    return return_status;
}

/////////////////////////
// IsShootToPOIBlocked //
/////////////////////////

bool isShootToPOIBlocked(const Point2D& poi, tpShootTypeEnum shootType)
{
    bool returnVal = false;

    double shootPathRadius = teamplay::ConfigurationStore::getConfiguration().getShootPathWidth() / 2.0;
    Point2D robotPos = teamplay::RobotStore::getInstance().getOwnRobot().getPosition();
    std::vector<Obstacle> obstacles;

    WorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, poi, shootPathRadius, obstacles);

    if (shootType == tpShootTypeEnum::LOB_SHOT)
    {
        // if obstacle far enough away, we assume we can lob shot over the obstacle
        double lobshotDistanceThreshold = ROBOT_RADIUS + 0.6; // TODO - make configurable

        // lobshot is blocked when an obstacle is closer to the robot than lobshotDistanceThreshold,
        // and if that obstacle is not the goalkeeper

        for (const auto& obst: obstacles)
        {
            bool obstacleTooClose = (obst.getDistanceTo(robotPos) < lobshotDistanceThreshold);

            Area2D oppGoalArea = teamplay::FieldDimensionsStore::getInstance().getFieldDimensions().getArea(teamplay::FieldArea::OPP_GOALAREA);
            bool obstacleInOppGoalArea = oppGoalArea.includesPosition( Position2D(obst.getLocation().x, obst.getLocation().y, 0.0) );

            if (obstacleTooClose && !obstacleInOppGoalArea)
            {
                returnVal = true;
            }
        }
    }
    else
    {
        returnVal = (obstacles.size() > 0);
    }
    
    return returnVal;
}

BT::NodeStatus IsShootToPOIBlocked::tick()
{
    const size_t children_count = children_nodes_.size();

    if (children_count != 2)
    {
        throw std::logic_error("IsShootToPOIBlocked must have 2 children");
    }

    setStatus(BT::NodeStatus::RUNNING);

    // Parse poi
    auto res = getInput<Point2D>("poi");
    if (!res)
    {
        throw BT::RuntimeError("error reading port [poi]:", res.error());
    }
    Point2D poi = res.value();

    // Parse shootType
    auto res2 = getInput<tpShootTypeEnum>("shootType");
    if (!res2)
    {
        throw BT::RuntimeError("error reading port [shootType]:", res.error());
    }
    tpShootTypeEnum shootType = res2.value();


    BT::NodeStatus return_status;

    bool result = isShootToPOIBlocked(poi, shootType);

    // Trace result
    std::ostringstream oss;
    oss << shootTypeMapping.right.at(shootType) << " to " << poi << " blocked: ";
    result ? oss << "true" : oss << "false";
    TRACE_FUNCTION(oss.str().c_str());

    if (result)
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

    _previousResult = result;

    return return_status;
}
