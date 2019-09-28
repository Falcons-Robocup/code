export TEXTDOMAIN=Linux-PAM

. gettext.sh

# Added XX by Andre on May 5, 2018 to prevent message when logging in

if [ -e /run/sshwarnXX ] ; then
    echo
	echo $(/usr/bin/gettext "SSH is enabled and the default password for the 'pi' user has not been changed.")
	echo $(/usr/bin/gettext "This is a security risk - please login as the 'pi' user and type 'passwd' to set a new password.")
	echo
fi
