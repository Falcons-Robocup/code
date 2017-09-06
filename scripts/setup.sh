#!/bin/bash

echo 'Adding your username to sudo list'
sudo adduser $USER sudo

echo 'Adding ROS groovy to source list'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net:80 --recv-key 0xB01FA116

echo 'Adding google chrome to source list'
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list'

echo 'Installing packages'
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y ros-jade-desktop-full python-rosinstall ros-jade-rosdoc-lite google-chrome-stable zlib1g-dev

sudo apt-get install -y `cat $HOME/falcons/code/config/apt_packages`

sudo pip install robotframework
sudo pip install pybindgen
sudo pip install Pyro4==4.39

echo 'Initializing ROS depedencies'
sudo rosdep init
rosdep update

echo 'checking out GIT repositories'
mkdir ~/falcons
cd ~/falcons
git clone ssh://git@git.falcons-robocup.nl:2222/falcons/data.git
git clone ssh://git@git.falcons-robocup.nl:2222/falcons/teamplayData.git

echo 'Adding GIT repo to ROS environment'
echo "# ROS packages lookup dirs" >> ~/.bashrc
echo "source $HOME/falcons/code/scripts/falconsconfig.sh" >> ~/.bashrc

echo 'Adding user to dialout to use serial communication'
sudo gpasswd --add robocup dialout
sudo usermod -a -G dialout robocup
sudo chmod 777 /dev/ttyS*
sudo chmod 777 /dev/ttyUSB*

echo 'Installing teamviewer'
sudo dpkg --add-architecture i386
sudo dpkg --install ~/falcons/data/external/teamviewer/teamviewer_10.0.46203_i386.deb
sudo apt-get install -f -y
sudo teamviewer --passwd robocupasml

echo 'Installing kalman'
mkdir -p ~/kalman
cd ~/kalman
unzip ~/falcons/data/external/kalman/kalman-1.3.zip
cd ~/kalman/kalman
make
sudo make install
echo "Kalman ready to be used"

echo 'Rerouting rc.local and hosts'
sudo rm /etc/rc.local
sudo ln -s ~/falcons/code/config/rc.local /etc/
sudo /etc/rc.local

echo 'Installing pygame'
mkdir ~/lib
cp ~/falcons/data/external/pygame/pgu*.zip ~/lib/
cd ~/lib
echo "unpacking PGU for visualizer to ~/lib/pgu"
unzip ~/lib/pgu*.zip > /dev/null
mv pgu-0.18 pgu

echo 'Installing Atmel avr-gcc (instead of the .deb avr-gcc)'
# The .deb version of avr-gcc results in a linking error that section .BOOT overlaps with section .data
sudo mkdir -p /opt
sudo rm -rf /opt/avr_v3.5.0
sudo tar xzf ~/falcons/data/external/avr/avr8-gnu-toolchain-3.5.0.1662-linux.any.x86_64.tar.gz -C /opt
sudo mv /opt/avr8-gnu-toolchain-linux_x86_64 /opt/avr_v3.5.0
sudo chown root.root -R /opt/avr_v3.5.0
sudo chmod go-w -R /opt/avr_v3.5.0

echo 'Running SSH key generator'
source ~/.profile
source ~/.bashrc
sh ~/falcons/code/scripts/genrsakey

echo 'Installing behavior3'
~/falcons/code/scripts/installBehavior3.sh
