// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header implemented in this file
#include "int/RoleAllocation.hpp"

// headers from this package
#include "int/Errors.hpp"

// system headers
#include <cmath>
#include <sstream>
#include <iomanip>

using namespace mtp;


void RoleAllocationAlgorithmBruteForce::_run()
{
    // check if current situation is OK
    if (currentIsOk())
    {
        result = _input.currentRoles;
        return;
    }
    // generate candidates
    auto candidates = generateCandidates(); // TODO caching?
    // pick the best one
    float bestPenalty = 1e9;
    for (auto const &candidate: candidates)
    {
        float penalty = calculatePenalty(candidate);
        if (penalty < bestPenalty)
        {
            bestPenalty = penalty;
            result = candidate;
        }
    }
    //printf("bestPenalty=%e\n", bestPenalty); //DEBUG
    // TODO random pick?
}

std::vector<RoleAllocation> RoleAllocationAlgorithmBruteForce::generateCandidates()
{
    // TODO: caching / profiling?
    std::vector<RoleAllocation> result;
    std::vector<RoleEnum> roles = allAssignableRoles();
    std::vector<PlayerId> players;
    for (auto const& imap: _input.currentRoles) players.push_back(imap.first);
    // initialize running RoleAllocation object
    RoleAllocation rc;
    for (auto const &player: players) rc[player] = roles.at(0);
    // construct cartesian product
    int P = players.size();
    int R = roles.size();
    int N = (int)round(pow(R, P));
    int c = 0;
    while (c < N)
    {
        // store
        result.push_back(rc);
        // increment
        int playerIdx = 0;
        int v = (++c) % R;
        int k = c;
        rc[players.at(playerIdx)] = RoleEnum(roles.at(v));
        while (v == 0 && playerIdx+1 < P) // carry
        {
            playerIdx++;
            k = k / R;
            v = k % R;
            rc[players.at(playerIdx)] = RoleEnum(roles.at(v));
        }
    }
    return result;
}

float RoleAllocationAlgorithmBruteForce::calculatePenalty(RoleAllocation const &candidate)
{
    // what makes a good candidate?
    //   * satisfy rules
    //   * according to own preference
    //   * similar to current
    auto count = roleAllocationToCount(candidate);
    bool validTeam = checkRoleCount(count); // TODO upstream, remove all invalid candidates earlier
    auto myRole = candidate.at(_input.myId);
    bool validSelf = checkRoleCount(myRole, count.at(myRole));
    bool preferred = (myRole == _input.preferredRoles.at(_input.myId).role);
    int difference = 0;
    for (auto const &rp: candidate)
    {
        difference += (_input.currentRoles.at(rp.first) != rp.second);
    }
    float penalty = 1000.0 * (!validTeam) + 100.0 * (!validSelf) + 1.0 * difference + 10.0 * !preferred;
    return penalty;
}
