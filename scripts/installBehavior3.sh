#!/bin/bash

sudo ln -s /usr/bin/nodejs /usr/bin/node

echo 'Install [phase 1] behavior3 dependency: nw-builder'
sudo npm install nw-builder -g

echo 'Install [phase 1] behavior3 dependency: bower'
sudo npm install bower -g

echo 'Install [phase 1] behavior3 dependency: gulp'
sudo npm install gulp -g

echo 'Install [phase 2] behavior3 dependency: bower'
cd ~/falcons/data/external/behavior3/
sudo npm install
bower install

echo 'Install [phase 2] behavior3 dependency: gulp'
sudo gulp dist

echo 'Install [phase 1] behavior3 dependency: n'
sudo npm cache clean -f
sudo npm install -g n

echo 'Install [phase 2] behavior3 dependency: n'
sudo n stable

