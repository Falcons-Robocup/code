*** Settings ***
Library         Process
Library         Collections
Library         OperatingSystem
Library         ROSWorldModel

Resource        heartBeat.robot

*** Keywords ***
WorldModelNode is started
    Own robot number is set to "1"
    Set Environment Variable  SIMULATED  1
    Set Environment Variable  ROS_IP  127.0.0.1
    Start Process  roscore                             shell=True	alias=roscore
    Start Process  rosrun worldModel WorldModelNode    shell=True	alias=WorldModel
    Sleep  2 seconds
    
HeartBeatNode is started
    The heartBeat node is initialized
    
WorldModelNode is restarted
    Terminate Process  WorldModel
    Start Process  rosrun worldModel WorldModelNode    shell=True  alias=WorldModel
    
Own robot number is set to "${ROBOT_NUMBER}"
    Set Environment Variable  TURTLE5K_ROBOTNUMBER  ${ROBOT_NUMBER}
    
Own robot position is set to "${ROBOT_X}" "${ROBOT_Y}" "${ROBOT_TH}"
	@{ROBOT_POS} =	Create List	${ROBOT_X}  ${ROBOT_Y}  ${ROBOT_TH}
    Set own robot position	${ROBOT_POS}
    World model cache is recalculated
    
Team robot "${ROBOT_NUMBER}" position is set to "${ROBOT_X}" "${ROBOT_Y}" "${ROBOT_TH}"
    @{ROBOT_POS} =  Create List  ${ROBOT_X}  ${ROBOT_Y}  ${ROBOT_TH}
    Set team robot position  ${ROBOT_NUMBER}  ${ROBOT_POS}
    World model cache is recalculated
    
Own robot position should be equal to "${ROBOT_X}" "${ROBOT_Y}" "${ROBOT_TH}"
    @{ROBOT_ABS_POS} =  Create List  ${ROBOT_X}  ${ROBOT_Y}  ${ROBOT_TH}
    @{ROBOT_POS}=  Get own robot location
    Should Be Equal As Numbers  @{ROBOT_POS}[0]  @{ROBOT_ABS_POS}[0]  precision=6
    Should Be Equal As Numbers  @{ROBOT_POS}[1]  @{ROBOT_ABS_POS}[1]  precision=6
    Should Be Equal As Numbers  @{ROBOT_POS}[2]  @{ROBOT_ABS_POS}[2]  precision=6
    
Team robot "${ROBOT_NUMBER}" position should be equal to "${ROBOT_X}" "${ROBOT_Y}" "${ROBOT_TH}"
    @{ROBOT_ABS_POS} =  Create List  ${ROBOT_X}  ${ROBOT_Y}  ${ROBOT_TH}
    @{ROBOT_POS}=  Get team robot location  ${ROBOT_NUMBER}
    Should Be Equal As Numbers  @{ROBOT_POS}[0]  @{ROBOT_ABS_POS}[0]  precision=6
    Should Be Equal As Numbers  @{ROBOT_POS}[1]  @{ROBOT_ABS_POS}[1]  precision=6
    Should Be Equal As Numbers  @{ROBOT_POS}[2]  @{ROBOT_ABS_POS}[2]  precision=6
    
Own robot velocity should be equal to "${ROBOT_VX}" "${ROBOT_VY}" "${ROBOT_VTH}"
    @{ROBOT_ABS_VEL} =  Create List  ${ROBOT_VX}  ${ROBOT_VY}  ${ROBOT_VTH}
    @{ROBOT_VEL}=  Get own robot velocity
    # Use precision 0 for velocity since timing issues with python calls can take time
    # Longer python calls means slower speed (time-divider will be larger)
    Should Be Equal As Numbers  @{ROBOT_VEL}[0]  @{ROBOT_ABS_VEL}[0]  precision=0
    Should Be Equal As Numbers  @{ROBOT_VEL}[1]  @{ROBOT_ABS_VEL}[1]  precision=0
    Should Be Equal As Numbers  @{ROBOT_VEL}[2]  @{ROBOT_ABS_VEL}[2]  precision=0
    
Team robot "${ROBOT_NUMBER}" velocity should be equal to "${ROBOT_VX}" "${ROBOT_VY}" "${ROBOT_VTH}"
    @{ROBOT_ABS_VEL} =  Create List  ${ROBOT_VX}  ${ROBOT_VY}  ${ROBOT_VTH}
    @{ROBOT_VEL}=  Get team robot velocity  ${ROBOT_NUMBER}
    # Use precision 0 for velocity since timing issues with python calls can take time
    # Longer python calls means slower speed (time-divider will be larger)
    Should Be Equal As Numbers  @{ROBOT_VEL}[0]  @{ROBOT_ABS_VEL}[0]  precision=0
    Should Be Equal As Numbers  @{ROBOT_VEL}[1]  @{ROBOT_ABS_VEL}[1]  precision=0
    Should Be Equal As Numbers  @{ROBOT_VEL}[2]  @{ROBOT_ABS_VEL}[2]  precision=0
    
The relative position of the ball is set to "${BALL_ANGLE}" "${BALL_RADIUS}" "${BALL_HEIGHT}"
	@{BALL_REL_POS} =	Create List  ${BALL_ANGLE}  ${BALL_RADIUS}  ${BALL_HEIGHT}
    Set own ball relative position	${BALL_REL_POS}
    World model cache is recalculated
    
The relative position of the ball of the front camera is set to "${BALL_ANGLE}" "${BALL_RADIUS}" "${BALL_DISTANCE}" "${CAMERA_HEIGHT}" "${CAMERA_FRONT_OFFSET}"
    @{BALL_REL_POS} =   Create List  ${BALL_ANGLE}  ${BALL_RADIUS}  ${BALL_DISTANCE}  ${CAMERA_HEIGHT}  ${CAMERA_FRONT_OFFSET}
    set own front camera ball relative position  ${BALL_REL_POS}
    World model cache is recalculated
    
The absolute position of the ball should be equal to "${BALL_X}" "${BALL_Y}" "${BALL_Z}"
	@{BALL_ABS_POS} =	Create List  ${BALL_X}  ${BALL_Y}  ${BALL_Z}
	@{BALL_POS} =  Get absolute ball position
    Should Be Equal As Numbers	@{BALL_POS}[0]	@{BALL_ABS_POS}[0]  precision=6
    Should Be Equal As Numbers  @{BALL_POS}[1]  @{BALL_ABS_POS}[1]  precision=6
    Should Be Equal As Numbers  @{BALL_POS}[2]  @{BALL_ABS_POS}[2]  precision=6
    
The length of the team members list should be equal to "${EXPECTED_LENGTH}"
    @{MEMBERS_LIST} =  Get all team members
    ${LIST_LENGTH} =  Get Length  ${MEMBERS_LIST}
    Should Be Equal As Numbers  ${LIST_LENGTH}  ${EXPECTED_LENGTH}
    
Robot ID "${ROBOT_NUMBER}" should not be present as team member
    @{MEMBER_IDS} =  Get team robots IDs
    Should Not Contain  ${MEMBER_IDS}  ${ROBOT_NUMBER}
    
The list of team members should be sorted on robotID
    @{MEMBER_IDS} =  Get team robots IDs
    ${PREV_ID} =  Set Variable  0
    :FOR  ${ID}  IN  @{MEMBER_IDS}
    \   Log  PREV_ID ${PREV_ID}
    \   Log  ID ${ID}
    \   Should be true  ${PREV_ID} < ${ID}
    \   ${PREV_ID} =  Set Variable  ${ID}
    
World model cache is recalculated
   force worldModel recalculation
    
World model balls are cleared
   Sleep  1100 milliseconds  # Ball time-out is 1 second