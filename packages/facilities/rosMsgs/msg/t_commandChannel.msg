# defines for commandType how to interpret the commandValue
uint16 INVALID = 0
#uint16 t_action = 1       		#commandValue = serialized t_action message   / obsolete with Teamplay v2
uint16 obstacleList = 2   		#             = serialized obstacleList
uint16 stringCommand = 3  		#             = plain string
uint16 controlInterfaceCommand = 4 	#             = string formatted for teamplay control interface
uint16 controlInterfaceAction  = 5 	#             = string formatted for teamplay control interface, separated by ';' for repeat specifiction ( "once" or "whileRunning" )
uint16 controlInterfaceBehavior= 6 	#             = string formatted for teamplay control interface
uint16 controlInterfaceRole    = 7 	#             = string formatted for teamplay control interface


# The actual command definition
string sender			# one char: C=Coach, 1-5 is robotnumber 
string to                       # one or multiple chars to indicate destination
string commandOwner		# who/which program/function uses it , more easy to filter commands for its purpose
uint16 commandType		# how to interpret the commandValue (one of the above defined uint16 values)
string commandValue     	# the command value, specific for the given commandType 
uint32 commandID		# used in case commandowner wants to provide a refererence number for commands
string checksum			# used by transport protocol, not part of the command
