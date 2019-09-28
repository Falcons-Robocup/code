*** Settings ****
Library         Process 
Resource        ../resources/FalconsControl.robot
Test Teardown   Shutting down



# PathPlanning_target.robot
#
# Created on: Jan 18, 2015
#     Author: Jan Feitsma
# Purpose: to check PathPlanning behavior.




*** Keywords ***

# To set target, wait a little while, then verify the result
Run Target check
    [Arguments]  ${ROBOT_X}  ${ROBOT_Y}  ${ROBOT_PHI}
   Given Simulator is started 
     And robot 1 of teamA is started
   When sleeping for 3 seconds
     And setting target of robot 1 of teamA to ${ROBOT_X} ${ROBOT_Y} ${ROBOT_PHI}
     And sleeping for 16 seconds
     And shutting down
   Then analyze logging
     And final worldmodel position of robot 1 of teamA is ${ROBOT_X} ${ROBOT_Y} ${ROBOT_PHI}
     And no errors occurred
    
*** Test Cases ***
Target checks
    [Template]  Run target check
   #X     Y     PHI
    0.0   0.0   0.0
    3.0   3.0   1.0
   -4.0  -4.0  -1.5

# This test case checks that robot will not move by itself (old zero target issue #4)
Initial stationary 
   Given Simulator is started 
     And robot 1 of teamA is started
   When sleeping for 12 seconds
     And shutting down
   Then analyze logging
     And final worldmodel position of robot 1 of teamA is -3.0 -5.0 0
     And no errors occurred

# This test will create an opponent (r2) and move it to 0,0.
# r1 will move from its inital position, -3 -5 0, to 3 5 0.
# Check to see if TrajectoryPlanning evades the opponent.
Trajectory - Evade opponent
   Given Simulator is started
      And robot 1 of teamA is started
      And robot 1 of teamB is started
   When sleeping for 3 seconds
      And setting target of robot 1 of teamA to 4.5 7.5 0.0
      And sleeping for 12 seconds
      And shutting down
   Then analyze logging
      And final worldmodel position of robot 1 of teamA is 4.5 7.5 0.0
      And no collisions occurred







