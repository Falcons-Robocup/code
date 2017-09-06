# Motion Profile enum
uint8 MOTIONPROFILE_NORMAL     = 0
uint8 MOTIONPROFILE_SETPIECE   = 1

# Move Type enum
uint8 MOVETYPE_NORMAL = 0

float32 x
float32 y
float32 phi
uint8   motion_profile
uint8   move_type
t_polygon2D[] forbiddenAreas
