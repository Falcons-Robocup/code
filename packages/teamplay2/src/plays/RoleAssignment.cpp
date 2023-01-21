// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "int/plays/RoleAssignment.hpp"

#include "int/stores/RobotStore.hpp"
#include "int/stores/GameStateStore.hpp"

#include "int/adapters/RTDBInputAdapter.hpp"
#include "int/adapters/RTDBOutputAdapter.hpp"

#include "tracing.hpp"

using namespace teamplay;

RoleAssignment::RoleAssignment()
:
    _self(createPlayerId(getTeamChar()))
{
}

void RoleAssignment::setBTFactory(BT::BehaviorTreeFactory& factory)
{
    _factory = &factory;
}

void RoleAssignment::setMTP(MixedTeamProtocolAdapter* mtp)
{
    _mtpAdapter = mtp;
}

T_ROBOT_ROLES convertRolesType(RoleAssignmentType const &roles)
{
    T_ROBOT_ROLES result;
    for (auto rolepair: roles)
    {
        int id = rolepair.first.shirtId;
        result.insert({id, rolepair.second});
    }
    return result;
}

void RoleAssignment::assignRoles()
{
    TRACE_FUNCTION("");
    RoleAssignmentType robotRoles;

    // As Mixed Team Protocol is still in its early stages, we're going to fall back on
    // our own deterministic role assignment for matches where MTP is not required
    if ((*_mtpAdapter)->isMixedTeam())
    {
        robotRoles = assignRolesFromMTP();
    }
    else
    {
        robotRoles = assignRolesFromTeamplayLeader();
    }

    // the roles need to be written at each tick, to prevent data going stale
    tpRTDBOutputAdapter::getInstance().setRoles(convertRolesType(robotRoles));
}

RoleAssignmentType RoleAssignment::assignRolesFromMTP()
{
    TRACE_FUNCTION("");
    RoleAssignmentType robotRoles;

    // determine the preferred role for this robot
    // With MixedTeamProtocol, this needs to happen every tick
    determineRolePreference();
    setRolePreferenceInMTP();

    // get all roles from MTP and fill in the data for our own robots
    // TODO: how will this work for other mixed-team robots? To which extent does teamplay require that data?
    robotRoles = getRolesFromMTP();

    // fallback for testing with only 1 or 2 robots
    if (robotRoles.size() == 0 || (robotRoles.count(_self) && robotRoles.at(_self) == "UNDEFINED"))
    {
        // TODO: make configurable? should that be done in Falcons or in MTP?
        if (robotRoles.size() == 1)
        {
            robotRoles[_self] = "ATTACKER_MAIN";
        }
        if (robotRoles.size() == 2)
        {
            std::vector<std::string> overruleRoles = {"GOALKEEPER", "ATTACKER_MAIN"};
            int idx = 0;
            for (auto &rp: robotRoles)
            {
                rp.second = overruleRoles.at(idx);
                idx++;
            }
        }
    }

    return robotRoles;
}

bool RoleAssignment::needUpdate()
{
    TRACE_FUNCTION("");
    // update roles when:
    // - gamestate changed (except setpiece prepare -> execute)
    // - [TODO] ball location known   -> unknown
    // - [TODO] ball location unknown -> known
    bool result = false;
    GameState gameState = GameStateStore::getInstance().getGameState();

    if (gameState != _theGameState && !gameState.isExecuteSetPiece())
    {
        _theGameState = gameState;
        result = true;
    }
    return result;
}

void RoleAssignment::determineRolePreference()
{
    TRACE_FUNCTION("");
    _havePreferredRole = false;
    _preferredRole = "";

    // check hardware capabilities of robot
    // TODO: generalize and put into yaml or something
    if (getRobotNumber() == 1)
    {
        _havePreferredRole = true;
        _preferredRole = "GOALKEEPER";
        _preferenceFactor = 1.0; // force
        return;
    }

    // here we could optionally set situational preference / overrules
    // like calculating if robot is closest to ball, then claim initiative (like ATTACKER_MAIN)
    // which would minimize robot driving time for a setpiece
    // alternatively, such could be implemented as part of MTP, to prevent all teams having to implement it
}

void RoleAssignment::setRolePreferenceInMTP()
{
    TRACE_FUNCTION("");
    if (_havePreferredRole)
    {
        TRACE("setting role preference: %s", _preferredRole.c_str());
        (*_mtpAdapter)->setPreferredOwnRole(_preferredRole, _preferenceFactor);
    }
}

RoleAssignmentType RoleAssignment::getRolesFromMTP()
{
    TRACE_FUNCTION("");
    // poke MTP to tick
    (*_mtpAdapter)->tick(rtime::now());
    // create return type
    RoleAssignmentType robotRoles;
    // fill in own role
    robotRoles.insert({_self, (*_mtpAdapter)->getOwnRole()});
    // fill in teammembers
    bool ownRobotsOnly = false;
    auto teammembers = (*_mtpAdapter)->getTeam(ownRobotsOnly);
    TRACE("got %d teammembers", (int)teammembers.size());
    for (auto teammember: teammembers)
    {
        TRACE("teammember %s gets role %s", teammember.id.describe().c_str(), teammember.role.c_str());
        robotRoles.insert({teammember.id, teammember.role});
    }
    traceRoleAssignment("role assignment calculated by MTP", robotRoles);
    return robotRoles;
}

RoleAssignmentType RoleAssignment::assignRolesFromTeamplayLeader()
{
    TRACE_FUNCTION("");
    RoleAssignmentType robotRoles;

    auto all_robots = teamplay::RobotStore::getInstance().getAllRobots();
    auto lowest_active_robot = std::min_element(all_robots.begin(), all_robots.end());

    // Initialize the robotRoles from the lowest active robotid.
    auto tpRobotRoles = tpRTDBInputAdapter::getInstance().getRobotRoles(lowest_active_robot->getNumber());
    for (auto const& role: tpRobotRoles)
    {
        robotRoles.insert({createPlayerId(role.first), enum2str(role.second)});
    }

    // Leader (lowest_active_robot) decides play, and writes new roles to RtDB
    if (lowest_active_robot->getNumber() == teamplay::RobotStore::getInstance().getOwnRobot().getNumber())
    {
        if (needUpdate())
        {
            robotRoles = calculateTeamplayRoleAssignment();
        }
    }

    return robotRoles;
}


RoleAssignmentType RoleAssignment::calculateTeamplayRoleAssignment() const
{
    TRACE_FUNCTION("");
    RoleAssignmentType robotRoles;

    auto play = teamplay::PlaySelection::getInstance().selectPlay();

    const std::string DECISIONTREE_PATH = pathToTeamplayDataRepo();
    BT::Tree tree;

    // Load the tree for the selected Play
    switch (play)
    {
        case PlayEnum::DEFAULT_PLAY:
        {
            tree = _factory->createTreeFromFile(DECISIONTREE_PATH + "/RoleAssignmentTree.xml");
            break;
        }
        case PlayEnum::INVALID:
        {
            throw std::runtime_error("Invalid PlayEnum");
        }
        default:
        {
            throw std::runtime_error("Unknown PlayEnum");
        }
    }


    // Tick tree for me and my teammates to determine all roles
    // At the end of the tree, the role is set.

    // Tick tree for me
    tree.tickRoot();

    const auto &ownRobot = teamplay::RobotStore::getInstance().getOwnRobot();
    robotRoles.insert({createPlayerId(ownRobot.getNumber()), enum2str(ownRobot.getRole())});

    auto teammates = teamplay::RobotStore::getInstance().getAllRobotsExclOwnRobot();
    for (const auto &teammate : teammates)
    {
        teamplay::RobotStore::getInstance().exchangeOwnRobotWith(teammate.getNumber());

        // Tick tree for teammate to determine role
        tree.tickRoot();

        // while 'exchanged', teammates role is my own
        auto teammateRole = teamplay::RobotStore::getInstance().getOwnRobot().getRole();

        robotRoles.insert({createPlayerId(teammate.getNumber()), enum2str(teammateRole)});

        teamplay::RobotStore::getInstance().undoExchange();
    }

    traceRoleAssignment("role assignment calculated by teamplay (before asking MTP)", robotRoles);
    return robotRoles;
}

void RoleAssignment::traceRoleAssignment(std::string const &message, RoleAssignmentType const &roles) const
{
    TRACE_FUNCTION(message.c_str());
    for (auto const &rp: roles)
    {
        TRACE("%s -> %s", rp.first.describe().c_str(), rp.second.c_str());
    }
}
