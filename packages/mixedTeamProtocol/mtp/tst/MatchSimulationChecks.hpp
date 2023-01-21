// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_TST_MATCHSIMULATIONCHECKS_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_TST_MATCHSIMULATIONCHECKS_HPP_

// headers from this package
#include "MatchSimulation.hpp"

// standard/system headers
#include <map>

// a simple simulation of a partial match, using multiple simulated robots
class MatchSimulationChecks
{
    public:
        MatchSimulationChecks(MatchSimulation &m);
        ~MatchSimulationChecks();

        // setters
        void setExpectedPosVel(mtp::PlayerId const &playerId, mtp::Pose const &position, mtp::Pose const &velocity);

        // checks, true means OK
        bool checkRoleAllocation() const; // basic checks for both teams
        bool checkRoleAllocation(char teamId, mtp::RoleAllocation const &expectedRoles) const;
        bool checkTeamMemberCount(char teamId, int expectedCount) const;
        bool checkWorldModelPosVel() const; // check that locations are consistent for the interpretation of each robot

    private:
        MatchSimulation &_m;
        std::map<mtp::PlayerId, mtp::Pose> _expectedPosition;
        std::map<mtp::PlayerId, mtp::Pose> _expectedVelocity;
};

#endif
