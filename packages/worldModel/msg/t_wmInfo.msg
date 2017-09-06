float32 ballX
float32 ballY
float32 ballZ
float32 ballVX
float32 ballVY
float32 ballVZ
float32 ballConfidence
float64 ballTimestamp
bool    isBallValid
#TODO: Last known ball location

rosMsgs/BallPossession possession
float32 possessionX
float32 possessionY

rosMsgs/s_robot_status robotStatus
uint8[] activeRobots
bool ownRobotIsActive

uint64 nrObstacleMeasurements
float32[] obstacleX
float32[] obstacleY
float32[] obstacleVX
float32[] obstacleVY
float32[] obstacleConfidence
float64[] obstacleTimestamp

float32 locationX
float32 locationY
float32 locationTheta
float32 locationVx
float32 locationVy
float32 locationVtheta
float64 locationTimestamp

uint64 nrOfTeamMembers
uint8[] teamMemberRobotID
float32[] teamMemberX
float32[] teamMemberY
float32[] teamMemberTheta
float32[] teamMemberVx
float32[] teamMemberVy
float32[] teamMemberVtheta
float64[] teamMemberTimestamp
