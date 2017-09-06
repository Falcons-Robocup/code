WorldModel_own_location.robot

 Created on: Nov 25, 2014
     Author: Tim Kouters

*** Settings ****
Documentation   These testcases verify behavior of the
...             own robot administration of the WorldModel
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
Resource        ../resources/WorldModel.robot

Test Setup  WorldModelNode is started
Test Teardown   Terminate All Processes 
    
*** Keywords ***
Run Robot location updating by own vision
    [Arguments]  ${ROBOT_X}  ${ROBOT_Y}  ${ROBOT_TH}  ${ROBOT_SLEEP}  ${ROBOT_X2}  ${ROBOT_Y2}  ${ROBOT_TH2}
    Given own robot position is set to "${ROBOT_X}" "${ROBOT_Y}" "${ROBOT_TH}"
     And Sleep  ${ROBOT_SLEEP}s
    When own robot position is set to "${ROBOT_X2}" "${ROBOT_Y2}" "${ROBOT_TH2}"
    Then own robot position should be equal to "${ROBOT_X2}" "${ROBOT_Y2}" "${ROBOT_TH2}" 
    
*** Test Cases ***
Template own robot location  
    [Template]  Run Robot location updating by own vision
   # Position      | Time  |  Position 2     
   #X     Y     TH     Sleep  X2    Y2    TH2   
    0.0   0.0   0.0    0.1    0.0   0.0   0.0   
    0.0   0.0   0.0    0.1    0.1   0.0   0.0  
    0.0   0.0   0.0    0.1   -0.1   0.0   0.0  
    0.1   0.0   0.0    0.1    0.1   0.0   0.0  
    0.1   0.0   0.0    0.1   -0.1   0.0   0.0  
   -0.1   0.0   0.0    0.1   -0.1   0.0   0.0  
   -0.1   0.0   0.0    0.1    0.1   0.0   0.0  
   # -- Verify moving robot on Y-axis  
    0.0   0.0   0.0    0.1    0.0   0.0   0.0   
    0.0   0.0   0.0    0.1    0.0   0.1   0.0   
    0.0   0.0   0.0    0.1    0.0  -0.1   0.0   
    0.0   0.1   0.0    0.1    0.0  -0.1   0.0   
    0.0   0.1   0.0    0.1    0.0   0.0   0.0   
    0.0  -0.1   0.0    0.1    0.0  -0.1   0.0  
    0.0  -0.1   0.0    0.1    0.0   0.1   0.0  
   # -- Verify moving robot on Theta-axis  
    0.0   0.0   0.0    0.1    0.0   0.0   0.0  
    0.0   0.0   0.0    0.1    0.0   0.0   0.1  
    0.0   0.0   0.0    0.1    0.0   0.0  -0.1  
    0.0   0.0   0.1    0.1    0.0   0.0   0.1  
    0.0   0.0  -0.1    0.1    0.0   0.0   0.1  
    0.0   0.0  -0.1    0.1    0.0   0.0  -0.1  
    0.0   0.0   0.1    0.1    0.0   0.0  -0.1  
    
    
