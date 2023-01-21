// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "MatchSimulation.hpp"
#include "MatchSimulationChecks.hpp"
#include "TestCase.hpp"
#include <vector>


// 4 simulated ticks should be enough time for robots to decide on a role allocation
// * after tick 1, robots are aware of each others existence
// * after tick 2, the leader is identified
// * after tick 3, the only leader shares the role allocation
// * after tick 4, all robots report that the roles are OK
#define NUM_TICKS_SETTLE (4)


class MatchTest : public mtp::TestCase { };

TEST_F(MatchTest, SingleRobotSingleTick)
{
    // setup
    MatchSimulation m;
    m.addRobot(mtp::PlayerId(1, 1, 'A'));

    // run
    m.advanceTick();

    // no checks - this is mainly a profiling test case
    // typical use (see also build.py --trace option, for automating all of this):
    // * make sure uftrace is installed
    // * compile with -pg
    // * run command:
    //   uftrace record build/mtp/matchTests --gtest_filter=MatchTest.SingleRobotSingleTick
    // * run command:
    //   uftrace dump --chrome > /tmp/uftrace.json
    // * start browser: google-chrome
    // * browse to chrome://tracing/
    // * click 'load' and select the uftrace json file
}

TEST_F(MatchTest, TwoMixedTeamsInitialPhase)
{
    // setup
    // 2+3 against 3+2
    MatchSimulation m;
    m.addRobot(mtp::PlayerId(1, 1, 'A'));
    m.addRobot(mtp::PlayerId(1, 2, 'A'));
    m.addRobot(mtp::PlayerId(1, 3, 'B'));
    m.addRobot(mtp::PlayerId(1, 4, 'B'));
    m.addRobot(mtp::PlayerId(1, 5, 'B'));
    m.addRobot(mtp::PlayerId(2, 1, 'B'));
    m.addRobot(mtp::PlayerId(2, 2, 'B'));
    m.addRobot(mtp::PlayerId(2, 3, 'A'));
    m.addRobot(mtp::PlayerId(2, 4, 'A'));
    m.addRobot(mtp::PlayerId(2, 5, 'A'));

    // run
    m.advanceTicks(NUM_TICKS_SETTLE);

    // assert
    MatchSimulationChecks t(m);
    EXPECT_TRUE(t.checkTeamMemberCount('A', 5));
    EXPECT_TRUE(t.checkTeamMemberCount('B', 5));
    EXPECT_TRUE(t.checkRoleAllocation());
}

void checkRolePreferences3v3(MatchSimulation &m)
{
    MatchSimulationChecks t(m);
    EXPECT_TRUE(t.checkTeamMemberCount('A', 3));
    EXPECT_TRUE(t.checkTeamMemberCount('B', 3));
    mtp::RoleAllocation expectedRolesTeamA;
    expectedRolesTeamA[mtp::PlayerId(1, 1, 'A')] = mtp::RoleEnum::ATTACKER_MAIN;
    expectedRolesTeamA[mtp::PlayerId(1, 2, 'A')] = mtp::RoleEnum::GOALKEEPER;
    expectedRolesTeamA[mtp::PlayerId(1, 3, 'A')] = mtp::RoleEnum::DEFENDER_MAIN;
    mtp::RoleAllocation expectedRolesTeamB;
    expectedRolesTeamB[mtp::PlayerId(1, 1, 'B')] = mtp::RoleEnum::DEFENDER_MAIN;
    expectedRolesTeamB[mtp::PlayerId(1, 2, 'B')] = mtp::RoleEnum::ATTACKER_MAIN;
    expectedRolesTeamB[mtp::PlayerId(1, 3, 'B')] = mtp::RoleEnum::GOALKEEPER;
    EXPECT_TRUE(t.checkRoleAllocation());
    EXPECT_TRUE(t.checkRoleAllocation('A', expectedRolesTeamA));
    EXPECT_TRUE(t.checkRoleAllocation('B', expectedRolesTeamB));
}

TEST_F(MatchTest, RolePreferences)
{
    // setup
    MatchSimulation m;
    m.addRobot(mtp::PlayerId(1, 1, 'A'));
    m.addRobot(mtp::PlayerId(1, 2, 'A')).setPreferredRole(mtp::RoleEnum::GOALKEEPER);
    m.addRobot(mtp::PlayerId(1, 3, 'A')).setPreferredRole(mtp::RoleEnum::DEFENDER_MAIN);
    m.addRobot(mtp::PlayerId(2, 1, 'B')).setPreferredRole(mtp::RoleEnum::DEFENDER_MAIN);
    m.addRobot(mtp::PlayerId(2, 2, 'B')).setPreferredRole(mtp::RoleEnum::ATTACKER_MAIN);
    m.addRobot(mtp::PlayerId(2, 3, 'B'));

    // run
    m.advanceTicks(NUM_TICKS_SETTLE);

    // assert
    checkRolePreferences3v3(m);
}

TEST_F(MatchTest, RoleCurrentStaysSame)
{
    // setup
    MatchSimulation m;
    m.addRobot(mtp::PlayerId(1, 1, 'A'));
    m.addRobot(mtp::PlayerId(1, 2, 'A')).setCurrentRole(mtp::RoleEnum::GOALKEEPER);
    m.addRobot(mtp::PlayerId(1, 3, 'A')).setCurrentRole(mtp::RoleEnum::DEFENDER_MAIN);
    m.addRobot(mtp::PlayerId(2, 1, 'B')).setCurrentRole(mtp::RoleEnum::DEFENDER_MAIN);
    m.addRobot(mtp::PlayerId(2, 2, 'B')).setCurrentRole(mtp::RoleEnum::ATTACKER_MAIN);
    m.addRobot(mtp::PlayerId(2, 3, 'B'));

    // run
    m.advanceTicks(NUM_TICKS_SETTLE);

    // assert
    checkRolePreferences3v3(m);
}

TEST_F(MatchTest, WorldModelPosVel)
{
    // setup
    // 2 against 2, no full team needed just to check administration
    MatchSimulation m;
    MatchSimulationChecks t(m);
    for (int id = 1; id <= 4; ++id)
    {
        int shirtId = 1 + (id - 1) % 2; // resp. 1, 2, 1, 2
        char teamId = id < 3 ? 'A' : 'B';
        auto p = mtp::PlayerId(1, shirtId, teamId);
        mtp::Pose pos(id, id);
        mtp::Pose vel(-id, -id);
        m.addRobot(p).setOwnPosVel(pos, vel, 1.0);
        t.setExpectedPosVel(p, pos, vel);
    }

    // run
    m.advanceTicks(2); // first tick SEND and second tick RECV are needed to ensure data arrives

    // assert
    EXPECT_TRUE(t.checkTeamMemberCount('A', 2));
    EXPECT_TRUE(t.checkTeamMemberCount('B', 2));
    EXPECT_TRUE(t.checkWorldModelPosVel()); // check that locations are consistent for the interpretation of each robot
}

TEST_F(MatchTest, RoleAllocationNegotationSingleTeamInvalidCurrentState)
{
    // bug as reported by Jurge from VDL-Robotsports on 2021-05-21

    // setup
    MatchSimulation m;
    m.addRobot(mtp::PlayerId(1, 1, 'A')).setCurrentRole(mtp::RoleEnum::ATTACKER_GENERIC);
    m.addRobot(mtp::PlayerId(1, 2, 'A')).setCurrentRole(mtp::RoleEnum::ATTACKER_ASSIST);
    m.addRobot(mtp::PlayerId(1, 3, 'A')).setCurrentRole(mtp::RoleEnum::ATTACKER_MAIN);
    m.addRobot(mtp::PlayerId(1, 4, 'A')).setCurrentRole(mtp::RoleEnum::ATTACKER_GENERIC);

    // run
    m.advanceTicks(NUM_TICKS_SETTLE); // let the robots learn of each others existence and possibly already settle on a role allocation

    // again force the given situation
    m.getRobot(mtp::PlayerId(1, 1, 'A')).setCurrentRole(mtp::RoleEnum::ATTACKER_GENERIC);
    m.getRobot(mtp::PlayerId(1, 2, 'A')).setCurrentRole(mtp::RoleEnum::ATTACKER_ASSIST);
    m.getRobot(mtp::PlayerId(1, 3, 'A')).setCurrentRole(mtp::RoleEnum::ATTACKER_MAIN);
    m.getRobot(mtp::PlayerId(1, 4, 'A')).setCurrentRole(mtp::RoleEnum::ATTACKER_GENERIC);
    m.advanceTicks(NUM_TICKS_SETTLE);

    // assert
    MatchSimulationChecks t(m);
    mtp::RoleAllocation expectedRolesA;
    EXPECT_TRUE(t.checkTeamMemberCount('A', 4));
    EXPECT_TRUE(t.checkTeamMemberCount('B', 0));
    EXPECT_TRUE(t.checkRoleAllocation());
}

// TODO: more tests: wm data exchange, refbox signals, preferred roles, jitter/randomness, ...

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
