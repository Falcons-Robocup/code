#++
#
# $Id: t_role_assignment.msg 1648 2015-05-26 18:14:13Z jmbm $
#
# Message for role assignments to be sent by multicast from coach to robots.
#
#--

# Symbolic constants for roles
# Use offset 1000
#uint16 NO_ROLE				= 1000
#uint16 GOALKEEPER			= 1001
#uint16 KICKER				= 1002
#uint16 RECEIVER				= 1003

# The actual role message sent
uint16  iRobot
string  role
#
# Taf!
#
