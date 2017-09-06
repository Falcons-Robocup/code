*** Settings ***
Library         Process
Library         BuiltIn
Library         OperatingSystem
Library         Test2FalconsControl

*** Keywords ***
Simulator is started
    initialize simulator

Simulator with ${TEAM_NAME} as friendly is started
    initialize simulator  ${TEAM_NAME}

Shutting down
    shutdown simulator
    Terminate All Processes 

Robot ${ROBOT_NUMBER} of ${TEAM_NAME} is started
    Execute control       activate ${TEAM_NAME} ${ROBOT_NUMBER}

Sleeping for ${NUM_SECONDS} seconds
    Sleep  ${NUM_SECONDS}
    
Analyze logging
    load logging
    
No errors occurred
    check no errors
    
No collisions occurred
    check no collisions

Finally ball is in possession of robot ${ROBOT_NUMBER} of ${TEAM_NAME}
    verify ball possession  -0.1  ${ROBOT_NUMBER}  ${TEAM_NAME}

Finally ball is on field
    verify ball position field  -0.1

Final ball position is ${POSITION}
    verify ball position  -0.1  ${POSITION}
    
Final ${POS_TYPE} position of robot ${ROBOT_NUMBER} of ${TEAM_NAME} is ${POSITION}
    verify position  -0.1  ${ROBOT_NUMBER}  ${TEAM_NAME}  ${POSITION}  ${POS_TYPE}

Setting target of robot ${ROBOT_NUMBER} of ${TEAM_NAME} to ${ROBOT_X} ${ROBOT_Y} ${ROBOT_PHI}
    set target   ${ROBOT_NUMBER}  ${TEAM_NAME}  ${ROBOT_X}  ${ROBOT_Y}  ${ROBOT_PHI}

Teleporting ball to ${BALL_X} ${BALL_Y} ${BALL_VX} ${BALL_VY}
    teleport ball   ${BALL_X}  ${BALL_Y}  ${BALL_VX}  ${BALL_VY}

