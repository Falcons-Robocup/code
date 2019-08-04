# Falcons Code Repository

This repository contains the code for the Falcons Robocup Platform.

It contains:

* source files e.g. .cpp
* include files e.g. .hpp
* build files e.g. Makefiles
* configuration files e.g. .yaml
* scripts e.g. for startUp

## Rules
Sorry we like to keep things open, but to keep the high performance some basic rules are needed.  
Do not add generated files to the repository  
Do not add large files to the repository (e.g. .png, .avi)  
Pushing a repository with files larger than *256kByte* will be blocked!  
(Current largest .cpp file is 62.5kByte and largest .yaml 12.2kByte)  

## Directory Structure (alphabetical order)

| directory | owner | description |
| --- |:---:| --- |
| config | all | configuration files e.g. field dimensions |
| packages/command | jfei | control scripting and user interface |
| packages/compass | | interface to compass and gyro |
| packages/facilities/common | | |
| packages/facilities/geometry | | |
| packages/facilities/pyBindings | tkov | pygen and pybind |
| packages/facilities/sharedTypes | jfei | as rosMsgs but non-ROS datatype counterparts |
| packages/facilities/rosMsgs | jfei | ROS messages used on ROS interfaces |
| packages/facilities/tracing | jfei | |
| packages/facilities/udp | | |
| packages/pathPlanning | ekpc | |
| packages/peripheralsInterface | pmbc | x86-64 software for interfacing with the peripherals |
| packages/power/m4api | | battery status |
| packages/refbox | jfei | official refbox applet (downloaded) |
| packages/shootPlanning | ekpc | |
| packages/simulator | | |
| packages/teamplay | ivma | |
| packages/vision | apox | |
| packages/visualizer | jfei | visualizer user interface |
| packages/worldModel | tkov | |
| packages/worldModelSync | tkov | |
| peripherals | pmbc | **non** x86-64 software for interface boards |
| peripherals/xmegaMotor | apox | software for interface boards e.g. the motor controllers |
| scrips | all | startup and helper scripts that do not belong/fit in a package |
| testing | | |

## Coding Styles
For both filenames and directories use **lower camelCase** e.g. myFirstRobot

## Configuration files
For readability **.yaml** files are preferred above .xml

## Minimize merge issues
To minimize merge issues (and to make commits understandable) use multiple small commits rather than one large commit

## Commit comments
To browse through the logging, use one liners starting with the subject followed by the functional change

## Tags
When using tags please start with your 4 char abbreviation

## Alias
Add often used commands to your .gitconfig file

* coa = commit -a -m
* foo = commit -a -m 'none'

