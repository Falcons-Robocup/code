# gradually increasing activity states
uint8 OFFLINE  = 0  # robot does not produce any data at all
uint8 ONLINE   = 1  # robot is online, robotControl is sending 'alive' messages, but software is not started
uint8 STARTING = 2  # robot software is initializing
uint8 ACTIVE   = 3  # software is initialized but robot is out of play
uint8 INPLAY   = 4  # software is initialized and robot is inplay
uint8 INGAME   = 5  # robot is listening to refbox commands

uint8[] state # index: robotId (so index 0 is a dummy)

