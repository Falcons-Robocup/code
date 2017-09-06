#!/bin/bash
#
# JFEI 2015-03-17 creation
#
# wrapper to work around outputwrapper2 messing up argument quotes..


sleep 5 # workaround for some init race condition...

rosservice call /simulator/spawn "name: '$1'"

