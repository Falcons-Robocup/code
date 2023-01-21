#!/bin/bash

# Use a slightly modified version of the rtdb configuration on robots which blacklists the multicam interface
# TODO: Configure this blacklist more elegantly
extraArgs=""
if [ -n "$TURTLE5K_ROBOTNUMBER" ] && [ ! "$TURTLE5K_ROBOTNUMBER" = 0 ]; then
    extraArgs="--config $FALCONS_CONFIG_PATH/rtdb2_configuration_robot.xml"
else
    extraArgs="--config $FALCONS_CONFIG_PATH/rtdb2_configuration_refbox.xml"
fi

frun rtdb comm/comm -a ${TURTLE5K_ROBOTNUMBER} -p /tmp/rtdb_refbox_A -n refboxListener $extraArgs
