WorldModel_ball_location_front_camera.robot

 Created on: May 3, 2016
     Author: Tim Kouters

*** Settings ****
Documentation   These testcases verify behavior of the
...             ball administration of the WorldModel with the 
...             front camera
...
...             Gherkin syntax used below is explained at
...             http://en.wikipedia.org/wiki/Behavior-driven_development
...
...             Stakeholders: 
...              - frontVision
...              - Teamplay
...              - Simulator
...              - BaseStation
Library         Process
Resource		../resources/WorldModel.robot

Test Setup      WorldModelNode is started 
Test Teardown   Terminate All Processes 
	
*** Keywords ***
Run Ball location updating by own front vision
	[Arguments]  ${ROBOT_X}  ${ROBOT_Y}  ${ROBOT_TH}  ${BALL_ANGLE}  ${BALL_RADIUS}  ${BALL_DISTANCE}  ${CAMERA_HEIGHT}  ${CAMERA_OFFSET}  ${BALL_X}  ${BALL_Y}  ${BALL_Z}
    Given own robot position is set to "${ROBOT_X}" "${ROBOT_Y}" "${ROBOT_TH}"
      And World model balls are cleared
     When the relative position of the ball of the front camera is set to "${BALL_ANGLE}" "${BALL_RADIUS}" "${BALL_DISTANCE}" "${CAMERA_HEIGHT}" "${CAMERA_OFFSET}"
     Then the absolute position of the ball should be equal to "${BALL_X}" "${BALL_Y}" "${BALL_Z}" 
    
*** Test Cases ***
Template own ball location front camera	
	[Template]	Run Ball location updating by own front vision
   # ROBOT        |     BALL REL                            |     BALL ABS    |
   # X    Y    TH    Angle  Radi  Dist  CamHeight  CamOffset  X     Y     Z
	0.0  0.0  0.0    0.0    0.0   0.0   0.5        0.0        0.0   0.0   0.5
	0.0  0.0  0.0    0.0    2.0   1.0   0.5        0.8        1.8  -2.0   0.5
#	0.0  0.0  0.0    90.0   2.0   2.0   0.5        0.7        2.7   0.0   2.5   rejected due to Z limit of 2.0m (cWorldModel.hpp _ABSOLUTE_BALL_HEIGHT)
	0.0  0.0  0.0   180.0   2.0   3.0   0.5        0.6        3.6   2.0   0.5
	0.0  0.0  0.0   270.0   2.0   4.0   0.5        0.5        4.5   0.0   0.0  #-1.5
   # -- Verify camera radius and angle turning 
    0.0  0.0  0.0    0.0    1.0   2.0   0.5        0.0        2.0  -1.0   0.5
    0.0  0.0  0.0    90.0   1.0   2.0   0.5        0.0        2.0   0.0   1.5
    0.0  0.0  0.0   180.0   1.0   2.0   0.5        0.0        2.0   1.0   0.5
    0.0  0.0  0.0   270.0   1.0   2.0   0.5        0.0        2.0   0.0   0.0  #-0.5
   # -- Verify moving robot on X-axis  
    1.0  0.0  0.0    0.0    0.0   0.0   0.5        0.0        1.0   0.0   0.5
    2.0  0.0  0.0    0.0    2.0   1.0   0.5        0.0        3.0  -2.0   0.5
#    3.0  0.0  0.0    90.0   2.0   2.0   0.5        0.0        5.0   0.0   2.5   rejected due to Z limit of 2.0m (cWorldModel.hpp _ABSOLUTE_BALL_HEIGHT)
    4.0  0.0  0.0   180.0   2.0   3.0   0.5        0.0        7.0   2.0   0.5
    5.0  0.0  0.0   270.0   2.0   4.0   0.5        0.0        9.0   0.0   0.0  #-1.5
   # -- Verify moving robot on Y-axis  
    0.0  1.0  0.0    0.0    0.0   0.0   0.5        0.0        0.0   1.0   0.5
    0.0  2.0  0.0    0.0    2.0   1.0   0.5        0.0        1.0   0.0   0.5
#    0.0  3.0  0.0    90.0   2.0   2.0   0.5        0.0        0.0   5.0   2.5   rejected due to Z limit of 2.0m (cWorldModel.hpp _ABSOLUTE_BALL_HEIGHT)
#    0.0  4.0  0.0   180.0   2.0   3.0   0.5        0.0       -2.0   7.0   0.5
#    0.0  5.0  0.0   270.0   2.0   4.0   0.5        0.0        0.0   9.0  -1.5
   # -- Verify moving robot on Theta-axis  
    0.0  0.0  0.0    0.0    2.0   1.0   0.5        0.0        1.0  -2.0   0.5
    0.0  0.0  90.0   0.0    0.0   0.0   0.5        0.0        0.0   0.0   0.5    
    0.0  0.0  90.0   0.0    2.0   1.0   0.5        0.0        2.0   1.0   0.5  
#    0.0  0.0  90.0   90.0   2.0   2.0   0.5        0.0        0.0   2.0   2.5   rejected due to Z limit of 2.0m (cWorldModel.hpp _ABSOLUTE_BALL_HEIGHT)
    0.0  0.0  90.0  180.0   2.0   3.0   0.5        0.0       -2.0   3.0   0.5
    0.0  0.0  90.0  270.0   2.0   4.0   0.5        0.0        0.0   4.0   0.0  #-1.5
	
    
