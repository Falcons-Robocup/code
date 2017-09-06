#
# $Id: t_action.msg 2045 2015-07-17 13:07:19Z rsop $
#
# Preserve action 0 as invalid action
uint16 TYPE_INVALID = 0
uint16 NO_ACTION = 1

# Use offset 1000 for basic actions
uint16 BSC_STOP = 1000
uint16 BSC_SIMSHOOT = 1001               #expects 0params
uint16 BSC_SHOOT = 1002                  #expects 4params: x,y,z,pwr
uint16 BSC_PASS = 1003                   #expects 4params: x,y,pwr, ??
uint16 BSC_MOVE_TO_POS = 1004            #expects 6 or 3 params: posX,posY,theta,velX,velY,velZ
uint16 BSC_SELF_PASS = 1005

# Use offset 10000 for advanced actions
uint16 ADV_GOALKEEPER = 10000
uint16 ADV_GO_TO_BALL = 10001
uint16 ADV_SHOOT_AT_GOAL = 10002
uint16 ADV_SHOOT_AT_TARGET = 10003
uint16 ADV_RECEIVE_PASS = 10004
uint16 ADV_PASS_BALL = 10005             #expects 4: x,y,z,pwr
uint16 ADV_PENALTY = 10006
uint16 ADV_STOP = 10007
uint16 ADV_SETPIECE_PREPARE = 10008      #expects 3: state,role,ballposition
uint16 ADV_LEAVEGOALAREA = 10009
uint16 ADV_SETPIECE_EXECUTE = 10010
uint16 ADV_DEFEND = 10011
uint16 ADV_SEARCH_BALL = 10012
uint16 ADV_SELF_PASS = 10013		 #expects 6 or 3params: posX,posY,theta,velX,velY,velZ
uint16 ADV_SHOOT_AT_GOAL_TOKIODRIFT = 10014
uint16 DEMO_FETCHBALL = 900
uint16 DEMO_TC2015 = 901
uint16 ADV_MOVE_TO_POS = 10020           #expects 6 or 3params: posX,posY,theta,velX,velY,velZ

# The actual action message sent
uint16 action
string[] parameters

