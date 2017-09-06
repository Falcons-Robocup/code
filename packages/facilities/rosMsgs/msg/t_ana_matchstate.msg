# maintain on coach the state of the match

float64 startTime             # when the logging started in seconds sinds the epoch
float64 currentTime           # in seconds since the epoch, so that elapsed is the delta w.r.t. startTime
string  state                 # [emptystring], firstHalf, halfTime, secondHalf, afterMatch
string  lastRefboxCommand     # interpreted, to be shown on top of visualizer
float64 lastRefboxAge         # so that we can hide it after a few seconds
uint16  goalsOwn
uint16  goalsOpponent

