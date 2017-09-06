# note: this message is compatible with teamplay v1 and teamplay v2 for backwards compatibility of matchlog playback

string      action
string      behavior
string      role
string      state          # the gamestate
string[]    trees          # the full stack of executed "trees", including the action that is executed

bool        aiming
float32     shootTargetX
float32     shootTargetY

# legacy attributes, specific for teamplay v1:
string      event
string      task
