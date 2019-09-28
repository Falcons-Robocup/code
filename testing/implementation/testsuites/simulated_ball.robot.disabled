*** Settings ****
Library         Process 
Resource        ../resources/FalconsControl.robot
Test Teardown   Shutting down



# simball.robot
#
# Created on: 2015-06-09
#     Author: Jan Feitsma
# Purpose: to check simulated ball behavior.




*** Keywords ***

# To set a teleport, wait a little while, then verify the result
Teleportation check
    [Arguments]  ${SET_X}  ${SET_Y}  ${SET_VX}  ${SET_VY}  ${GET_X}  ${GET_Y}
   Given Simulator is started 
     And robot 1 of teamA is started
   When sleeping for 3 seconds
     And setting target of robot 1 of teamA to -3 -5 0
     And sleeping for 3 seconds
     And teleporting ball to ${SET_X} ${SET_Y} ${SET_VX} ${SET_VY}
     And sleeping for 10 seconds
     And shutting down
   Then analyze logging
     And final ball position is ${GET_X} ${GET_Y}
     And no errors occurred



*** Test Cases ***

# This test case checks that the ball is spawned at (0,0)
Initial position zero
   Given Simulator is started 
     And robot 1 of teamA is started
   When sleeping for 12 seconds
     And shutting down
   Then analyze logging
     And final ball position is 0.0 0.0
     And no errors occurred

# This test case checks that the ball can be teleported without speed
Teleport without speed
   Given Simulator is started 
     And robot 1 of teamA is started
   When sleeping for 12 seconds
     And teleporting ball to 3.0 3.0 -10.0 -10.0
     And sleeping for 3 seconds
     And shutting down
   Then analyze logging
     And final ball position is 3.0 3.0
     And no errors occurred

# Check that the ball bounces when a robot drives into it
# Issue #169 - this currently fails
#Robot moving into static ball
#   Given Simulator is started 
#     And robot 1 of teamA is started
#   When sleeping for 2 seconds
#     And setting target of robot 1 of teamA to -2 -0.1 3
#     And sleeping for 6 seconds
#     And setting target of robot 1 of teamA to 2 -0.1 3
#     And sleeping for 4 seconds
#     And shutting down
#   Then analyze logging
#     And final ball position is 3.0 3.0
#     And no errors occurred



*** Test Cases ***
Teleportation and bouncing checks
    [Template]  Teleportation check
 #SET_X SET_Y SET_VX SET_VY  GET_X  GET_Y
    0.0   1.0   10.0    0.0  -4.03   1.00  # top side
    0.0  -1.0  -10.0    0.0   4.03  -1.00  # bottom side
    0.0   0.0    0.0   10.0   0.00   0.00  # right goal backside
    0.0   0.0    0.0  -10.0   0.00   0.00  # left goal backside
    4.0   6.0    4.0    4.0   3.54   7.54  # right bottom corner
   -4.0   6.0   -4.0    4.0  -3.54   7.54  # right top corner
    4.0  -6.0    4.0   -4.0   3.54  -7.54  # left bottom corner
   -4.0  -6.0   -4.0   -4.0  -3.54  -7.54  # left top corner
   -3.0  -7.0    0.0    3.0  -3.01  -9.16  # bouncing off robot left side
   -3.0  -1.0    0.0   -3.0  -3.01  -2.65  # bouncing off robot right side
   -5.0  -5.0    3.0    0.0  -7.17  -5.15  # bouncing off robot back side


