WorldModel_teammembers_location.robot

 Created on: Nov 15, 2014
     Author: Tim Kouters

*** Settings ****
Documentation   These testcases verify behavior of the
...             team member administration of the WorldModel
...
...             Gherkin syntax used below is explained at
...             http://en.wikipedia.org/wiki/Behavior-driven_development
...
...             Stakeholders: 
...              - Teamplay
...              - Simulator
...              - BaseStation
Library         Process
Resource        ../resources/WorldModel.robot

Test Setup      WorldModelNode is started
Test Teardown   Terminate All Processes 

*** Keywords ***
Run Team member position administration
    [Arguments]  ${ROBOT2_X}  ${ROBOT2_Y}  ${ROBOT2_TH}  ${ROBOT_SLEEP}  ${ROBOT2_X2}  ${ROBOT2_Y2}  ${ROBOT2_TH2}
   Given team robot "2" position is set to "${ROBOT2_X}" "${ROBOT2_Y}" "${ROBOT2_TH}"
     And Sleep  ${ROBOT_SLEEP}s
    When team robot "2" position is set to "${ROBOT2_X2}" "${ROBOT2_Y2}" "${ROBOT2_TH2}"
    Then team robot "2" position should be equal to "${ROBOT2_X2}" "${ROBOT2_Y2}" "${ROBOT2_TH2}"
    
*** Test Cases ***
Template team member position and velocity  
    [Template]  Run Team member position administration
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
    
    
*** Test Cases ***
Own location not returned when asked for team members
   Given own robot number is set to "5"
     And WorldModelNode is restarted
     And team robot "3" position is set to "1.0" "1.0" "1.0"
     And team robot "4" position is set to "3.0" "3.0" "3.0"
     And own robot position is set to "6.0" "6.0" "6.0"
    When team robot "5" position is set to "8.0" "8.0" "8.0"
    Then the length of the team members list should be equal to "2"
     And robot ID "5" should not be present as team member
     
Team members are sorted on robotID
   Given own robot number is set to "2"
     And WorldModelNode is restarted
    When team robot "3" position is set to "3.0" "3.0" "3.0"
     And team robot "1" position is set to "1.0" "1.0" "1.0"
     And team robot "5" position is set to "5.0" "5.0" "5.0"
     And team robot "2" position is set to "2.0" "2.0" "2.0"
    Then the list of team members should be sorted on robotID
    