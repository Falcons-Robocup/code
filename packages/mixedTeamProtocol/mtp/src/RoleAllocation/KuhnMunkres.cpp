// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
// header partially implemented in this file
#include "int/RoleAllocation.hpp"

// headers from this package
#include "int/Errors.hpp"

// headers from other packages
#include "tprintf.hpp"

// system headers
#include <cmath>
#include <sstream>
#include <iomanip>

// Kuhn-Munkres library
#include "munkres.h"


using namespace mtp;


void RoleAllocationAlgorithmKuhnMunkres::_run()
{

    // we have P players and R roles
    std::vector<PlayerId> players;
    for (auto const &rp: _input.currentRoles)
    {
        players.push_back(rp.first);
    }
    int P = (int)players.size(); // typically 5, may grow towards 11 in future
    auto roles = allAssignableRoles();
    int R = (int)roles.size(); // about 8

    /* From Munkres documentation:
     *
     *     Linear assignment problem solution
     *     [modifies matrix in-place.]
     *     matrix(row,col): row major format assumed.
     *
     *     Assignments are remaining 0 values
     *     (extra 0 values are replaced with -1)
     *
     */

    // create and fill matrix
    float MATRIX_WEIGHT_DEFAULT = 20.0;
    float MATRIX_WEIGHT_DISABLED = 99.0;
    float MATRIX_WEIGHT_REQUIRED = 5.0; // to ensure the required roles (GOALKEEPER etc) are always chosen
    float MATRIX_WEIGHT_CURRENT_ROLE = -1.0; // give priority to current role, minimize switching
    float MATRIX_WEIGHT_PREFERRED_ROLE = -3.0; // give priority to preferred role
    float MATRIX_WEIGHT_OFFSET_ROLE_INDEX = 0.1; // give small priority to defined roles
    Matrix<float> m(P, R);
    for (int r = 0; r < R; ++r)
    {
        int minCount = 0, maxCount = 0;
        getMinMaxRoleCount(roles.at(r), minCount, maxCount);
        if (minCount > 1)
        {
            throw std::runtime_error("this algorithm does not yet support general role count constraints -- consider to use LinearProgramming instead");
            // or we could apply an iterative algorithm here?
        }
        for (int p = 0; p < P; ++p)
        {
            m(p,r) = MATRIX_WEIGHT_DEFAULT;
            if (minCount == 1)
            {
                m(p,r) = MATRIX_WEIGHT_REQUIRED;
            }
            if (maxCount == 0)
            {
                m(p,r) = MATRIX_WEIGHT_DISABLED;
            }
            m(p,r) += MATRIX_WEIGHT_OFFSET_ROLE_INDEX * r;
            if (roles.at(r) == _input.currentRoles.at(players.at(p)))
            {
                m(p,r) += MATRIX_WEIGHT_CURRENT_ROLE;
            }
            auto preferredRole = _input.preferredRoles.at(players.at(p)).role;
            if (preferredRole != RoleEnum::UNDEFINED && roles.at(r) == preferredRole)
            {
                m(p,r) += MATRIX_WEIGHT_PREFERRED_ROLE;
            }
        }
    }

    // solve
    //std::cout << "BEFORE SOLVE" << m << std::endl;
    Munkres<float> solver;
    solver.solve(m);
    //std::cout << "AFTER SOLVE" << m << std::endl;

    // check and convert solution
    result.clear();
    float EPSILON = 1e-8;
    for (int p = 0; p < P; ++p)
    {
        for (int r = 0; r < R; ++r)
        {
            if (fabs(m(p,r)) < EPSILON)
            {
                result[players.at(p)] = roles.at(r);
            }
        }
    }
}


