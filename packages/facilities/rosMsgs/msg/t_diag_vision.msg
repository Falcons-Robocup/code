float32      fps
int32        linePoints
int32        numBalls
int32        numObstacles

# below: legacy of vision+worldModel V1
float32      age # in seconds
float32      lastActive # in seconds
posconf      ownpos
pos          posnoise # stddev on (x,y,Rz)
# raw vision is not in FCS but relative coordinates 
# (WorldModel does the transformations)
angradconf[] ballpos
angradconf[] obstacles

