WorldModel_ball_location.robot

 Created on: Nov 15, 2014
     Author: Tim Kouters

*** Settings ****
Documentation   These testcases verify behavior of the
...             ball administration of the WorldModel
...
...             Gherkin syntax used below is explained at
...             http://en.wikipedia.org/wiki/Behavior-driven_development
...
...             Stakeholders: 
...              - WorldSensing
...              - Teamplay
...              - Simulator
...              - BaseStation
Library         Process
Resource		../resources/WorldModel.robot

Test Setup      WorldModelNode is started 
Test Teardown   Terminate All Processes 
	
*** Keywords ***
Run Ball location updating by own vision
	[Arguments]  ${ROBOT_X}  ${ROBOT_Y}  ${ROBOT_TH}  ${BALL_ANGLE}  ${BALL_RADIUS}  ${BALL_HEIGHT}  ${BALL_X}  ${BALL_Y}  ${BALL_Z}
    Given own robot position is set to "${ROBOT_X}" "${ROBOT_Y}" "${ROBOT_TH}"
      And World model balls are cleared
     When the relative position of the ball is set to "${BALL_ANGLE}" "${BALL_RADIUS}" "${BALL_HEIGHT}"
     Then the absolute position of the ball should be equal to "${BALL_X}" "${BALL_Y}" "${BALL_Z}" 
    
*** Test Cases ***
Template own ball location	
	[Template]	Run Ball location updating by own vision
   # ROBOT        |     BALL REL     |     BALL ABS    |
   # X    Y    TH     Angle  Radi  Z     X     Y     Z
	0.0  0.0  0.0    0.0    2.0   0.0   0.0  -2.0   0.0
	0.0  0.0  0.0    90.0   1.0   0.0   1.0   0.0   0.0
	0.0  0.0  0.0    180.0  3.0   0.0   0.0   3.0   0.0
	0.0  0.0  0.0    270.0  4.0   0.0  -4.0   0.0   0.0
	0.0  0.0  90.0   0.0    1.0   0.0   1.0   0.0   0.0
    0.0  0.0  180.0  0.0    3.0   0.0   0.0   3.0   0.0
    0.0  0.0  270.0  0.0    4.0   0.0  -4.0   0.0   0.0
   # -- Verify moving robot on X-axis  
    1.0  0.0  0.0    0.0    2.0   0.0   1.0  -2.0   0.0
    2.0  0.0  0.0    90.0   1.0   0.0   3.0   0.0   0.0
    3.0  0.0  0.0    180.0  3.0   0.0   3.0   3.0   0.0
    4.0  0.0  0.0    270.0  4.0   0.0   0.0   0.0   0.0
    5.0  0.0  90.0   0.0    1.0   0.0   6.0   0.0   0.0
    6.0  0.0  180.0  0.0    3.0   0.0   6.0   3.0   0.0
    7.0  0.0  270.0  0.0    4.0   0.0   3.0   0.0   0.0
   # -- Verify moving robot on Y-axis  
    0.0  1.0  0.0    0.0    2.0   0.0   0.0  -1.0   0.0
    0.0  2.0  0.0    90.0   1.0   0.0   1.0   2.0   0.0
    0.0  3.0  0.0    180.0  3.0   0.0   0.0   6.0   0.0
    0.0  4.0  0.0    270.0  4.0   0.0  -4.0   4.0   0.0
    0.0  5.0  90.0   0.0    1.0   0.0   1.0   5.0   0.0
    0.0  6.0  180.0  0.0    3.0   0.0   0.0   9.0   0.0
    0.0  7.0  270.0  0.0    4.0   0.0  -4.0   7.0   0.0
   # -- Verify moving ball on Z-axis  
    0.0  1.0  0.0    0.0    2.0   1.0   0.0  -1.0   1.0
    0.0  2.0  0.0    90.0   1.0   2.0   1.0   2.0   2.0
    0.0  3.0  0.0    180.0  3.0   3.0   0.0   6.0   3.0
    0.0  4.0  0.0    270.0  4.0   4.0  -4.0   4.0   4.0
    0.0  5.0  90.0   0.0    1.0   5.0   1.0   5.0   5.0
    0.0  6.0  180.0  0.0    3.0   6.0   0.0   9.0   6.0
    0.0  7.0  270.0  0.0    4.0   7.0  -4.0   7.0   7.0
	
    
