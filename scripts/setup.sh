#!/bin/bash

# NOTE: this script is intended for standardized laptop setup, so it currently is not particularly robust beyond this use case


# do not silently continue when encountering an error
set -e 

# Poor man's argument parsing, which is sufficient for now
if [[ "$*" == *"--skip-interactive"*     ]]; then
    SKIP_INTERACTIVE=1
fi

echo 'Adding your username to sudo list'
sudo adduser $USER sudo

# allow robocup to run dmidecode without password use
if ! sudo grep --quiet '/usr/sbin/dmidecode' /etc/sudoers; then
    echo '%robocup ALL=(ALL) NOPASSWD: /usr/sbin/dmidecode' | sudo EDITOR='tee -a' visudo
fi

echo 'Installing packages'
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y `cat $HOME/falcons/code/config/apt_packages`

echo 'Checking out GIT repositories'
eval $(ssh-agent)
ssh-add ~/.ssh/id_rsa_falconsguest
[ ! -d ~/falcons ] && mkdir ~/falcons
cd ~/falcons
[ ! -d ~/falcons/code ] && git clone ssh://git@git.falcons-robocup.nl:2222/falcons/code.git # probably already done beforehand
[ ! -d ~/falcons/data ] && git clone ssh://git@git.falcons-robocup.nl:2222/falcons/data.git # this one takes a while
[ ! -d ~/falcons/teamplayData ] && git clone ssh://git@git.falcons-robocup.nl:2222/falcons/teamplayData.git

# Try to update the repositories to the latest commits, but only if the workspaces are clean
git -C ~/falcons/code diff | grep -q '' || git -C ~/falcons/code pull
git -C ~/falcons/data diff | grep -q '' || git -C ~/falcons/data pull
git -C ~/falcons/teamplayData diff  | grep -q '' || git -C ~/falcons/teamplayData pull

echo 'Setup environment'
~/falcons/code/scripts/setupEnvBashrcHook.sh
source ~/falcons/code/scripts/setupEnv.sh

# Execute only on Robot to allow connection to ioBoard
echo 'Adding user to dialout to use serial communication'
~/falcons/code/scripts/onRobot && sudo gpasswd --add robocup dialout
~/falcons/code/scripts/onRobot && sudo usermod -a -G dialout robocup
~/falcons/code/scripts/onRobot && sudo chmod 777 /dev/ttyS*
~/falcons/code/scripts/onRobot && sudo chmod 777 /dev/ttyUSB* || true # robot specific

# TODO: this might not be a very nice thing to do for custom user laptops -> make more flexible?
echo 'Rerouting rc.local and hosts'
if [ -f /etc/rc.local ]; then
    sudo rm /etc/rc.local
fi
sudo ln -s ~/falcons/code/config/rc.local /etc/
sudo /etc/rc.local

echo 'Installing Atmel avr-gcc (instead of the .deb avr-gcc)'
# The .deb version of avr-gcc results in a linking error that section .BOOT overlaps with section .data
sudo mkdir -p /opt
sudo rm -rf /opt/avr_v3.5.0
sudo tar xzf ~/falcons/data/external/avr/avr8-gnu-toolchain-3.5.0.1662-linux.any.x86_64.tar.gz -C /opt
sudo mv /opt/avr8-gnu-toolchain-linux_x86_64 /opt/avr_v3.5.0
sudo chown root.root -R /opt/avr_v3.5.0
sudo chmod go-w -R /opt/avr_v3.5.0

echo 'Linking SSH authorized_keys'
cd ~/.ssh
[ ! -h authorized_keys ] && ln -s ../falcons/code/config/authorized_keys

echo 'Linking disk cleanup service'
cd /etc/systemd/system/
[ ! -h diskCleanup.service ] && sudo ln -s ~/falcons/code/config/diskCleanup.service
sudo systemctl enable diskCleanup.service

echo 'Installing git hooks'
cd ~/falcons/code/.git/hooks
[ ! -h pre-commit ] && ln -s ../../scripts/pre-commit
[ ! -h commit-msg ] && ln -s ../../scripts/commit-msg

echo 'Installing rtdb dependencies'
cd ~/falcons/code/packages/facilities/rtdb
./setup.sh

echo 'Installing pip3 packages'
pip3 install `cat $HOME/falcons/code/config/pip3_packages`

echo 'Installing refbox application'
~/falcons/code/scripts/setupRefbox.sh

echo 'Installing ripgrep utility'
sudo snap install ripgrep --classic

echo 'Installing catapult (external submodule in data repo)'
cd ~/falcons/data ; git submodule update --init ; cd -

echo 'Installing google-chrome'
cd /tmp
wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
sudo gdebi -n google-chrome-stable_current_amd64.deb

#echo 'Installing behavior3'
#~/falcons/code/scripts/installBehavior3.sh
# TODO needs fixing for ubuntu20 - in particular the npm requirement is problematic, as it depends on python2.7 and much more old crap


# Register the laptops (requires a configured git user)
# Note: this will not register the laptops until the file is available on the master branch
if ! ~/falcons/code/scripts/onRobot && [ ! ${SKIP_INTERACTIVE} ]; then

    # Only configure the user when this is not the case yet
    if ! ~/falcons/code/scripts/setLaptopUser --check; then
        echo
        echo '================'
        echo 'Configuring your git details'
        echo
        ~/falcons/code/scripts/setLaptopUser --skip-registration
    fi

    # Don't proceed if setting up the user has failed
    if ~/falcons/code/scripts/setLaptopUser --check; then
        echo
        echo '================'
        echo 'Configuring laptop hostname'
        echo
        ~/falcons/code/scripts/setupLaptopInfo.py
    fi
fi


cd


