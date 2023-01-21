#!/bin/bash

extraArgs=""

# Use a slightly modified version of the rtdb configuration on robots which blacklists the multicam interface
# TODO: Configure this blacklist more elegantly
if [ -n "$TURTLE5K_ROBOTNUMBER" ] && [ ! "$TURTLE5K_ROBOTNUMBER" = 0 ]; then
    extraArgs="--config $FALCONS_CONFIG_PATH/rtdb2_configuration_robot.xml"
fi

frun rtdb comm/comm -a ${TURTLE5K_ROBOTNUMBER} -p /tmp/rtdb_mixedteam_A -n MTP $extraArgs
