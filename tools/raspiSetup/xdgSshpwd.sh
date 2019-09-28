#!/bin/bash

export TEXTDOMAIN=pprompt

. gettext.sh

# Added XX by Andre on May 5, 2018 to prevent the unsecure warning on the display
if [ -e /run/sshwarnXX] ; then
	zenity --warning --no-wrap --text="$(gettext "SSH is enabled and the default password for the 'pi' user has not been changed.\nThis is a security risk - please login as the 'pi' user and run Raspberry Pi Configuration to set a new password.")"
fi
