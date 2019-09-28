# Falcons Code Repository

This repository contains the code for the Falcons MSL Robocup Platform.

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
Use other repositories (teamplayData, data, matchLogs, etc.) where applicable  

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

