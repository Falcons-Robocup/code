#!/bin/bash

# do not silently continue when encountering an error
set -e 


echo 'Adding your username to sudo list'
sudo adduser $USER sudo

echo 'Adding ROS groovy to source list'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

echo 'Adding google chrome to source list'
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list'

echo 'Installing packages'
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y ros-kinetic-desktop-full python-rosinstall ros-kinetic-rosdoc-lite zlib1g-dev python-rosinstall-generator python-wstool build-essential

sudo apt-get install -y `cat $HOME/falcons/code/config/apt_packages`

echo 'Initializing ROS dependencies'
sudo rosdep init
rosdep update

echo 'Running SSH key generator'
source ~/.profile
source ~/.bashrc
sh ~/falcons/code/scripts/genrsakey

echo 'checking out GIT repositories'
[ ! -d ~/falcons ] && mkdir ~/falcons
cd ~/falcons
[ ! -d ~/falcons/code ] && git clone http://git.falcons-robocup.nl/falcons/code.git # probably already done beforehand
git clone http://git.falcons-robocup.nl/falcons/data.git
git clone http://git.falcons-robocup.nl/falcons/teamplayData.git

echo 'Adding GIT repo to ROS environment'
echo "# ROS packages lookup dirs" >> ~/.bashrc
echo "source $HOME/falcons/code/scripts/falconsconfig.sh" >> ~/.bashrc

echo 'Adding user to dialout to use serial communication'
sudo gpasswd --add robocup dialout
sudo usermod -a -G dialout robocup
sudo chmod 777 /dev/ttyS*
sudo chmod 777 /dev/ttyUSB* || true # robot specific

echo 'Rerouting rc.local and hosts'
sudo rm /etc/rc.local
sudo ln -s ~/falcons/code/config/rc.local /etc/
sudo /etc/rc.local || true # not sure why we need to ignore result here ...

echo 'Installing Atmel avr-gcc (instead of the .deb avr-gcc)'
# The .deb version of avr-gcc results in a linking error that section .BOOT overlaps with section .data
sudo mkdir -p /opt
sudo rm -rf /opt/avr_v3.5.0
sudo tar xzf ~/falcons/data/external/avr/avr8-gnu-toolchain-3.5.0.1662-linux.any.x86_64.tar.gz -C /opt
sudo mv /opt/avr8-gnu-toolchain-linux_x86_64 /opt/avr_v3.5.0
sudo chown root.root -R /opt/avr_v3.5.0
sudo chmod go-w -R /opt/avr_v3.5.0

echo 'Installing behavior3'
~/falcons/code/scripts/installBehavior3.sh

echo 'Linking SSH authorized_keys'
cd ~/.ssh
[ ! -h authorized_keys ] && ln -s ../falcons/code/config/authorized_keys

echo 'Installing git hooks'
cd ~/falcons/code/.git/hooks
ln -s ../../scripts/pre-commit
ln -s ../../scripts/commit-msg

echo 'Installing ripgrep (rg)'
sudo ln -s /home/robocup/falcons/data/external/ripgrep/rg2017 /usr/local/bin/rg

echo 'Installing rtdb dependencies'
cd ~/falcons/code/packages/facilities/rtdb3
./setup.sh
if [ ! -f /usr/local/lib/python2.7/dist-packages/shm-1.2.2.egg-info ]; then
    cd /home/robocup/falcons/data/external/shm-python/shm-1.2.2
    sudo python setup.py install
fi

echo 'Installing pip packages'
sudo pip install `cat $HOME/falcons/code/config/pip_packages`

cd


