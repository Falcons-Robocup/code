// Copyright 2016 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Integration tests for TableViewModel.
 * TableViewModelTests.cpp
 *
 *  Created on: July 3, 2016
 *      Author: Diana Koenraadt
 */
#include <gtest/gtest.h>

#include "int/TeamRobotSelection.h"

class MyTeamRobotSelection : public TeamRobotSelection
{
public:
    bool teamModeChanged = false;
    bool robotModeChanged = false;
protected:
    virtual void onTeamModeChanged() 
    {
        teamModeChanged = true;
    };
    virtual void onRobotModeChanged() 
    {
        robotModeChanged = true;
    };
};

TEST(TeamRobotSelection, setTeamMode_callsOnTeamModeChanged)
{
    // Arrange
    MyTeamRobotSelection selection;

    // Act 
    selection.setTeamMode();

    // Assert
    ASSERT_TRUE(selection.teamModeChanged);
}

TEST(TeamRobotSelection, setRobotMode_callsOnRobotModeChanged)
{
    // Arrange
    MyTeamRobotSelection selection;

    // Act 
    selection.setRobotMode(2);

    // Assert
    ASSERT_TRUE(selection.robotModeChanged);
}

/*
 * Main entry
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
