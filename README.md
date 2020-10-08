# Falcons Code Repository

This repository contains the code for the Falcons MSL Robocup Platform.

## Contents

It contains:
* source and header files (mostly C++) for our functional packages
* main functional packages:
    * teamplay (engine only, not tactical content)
    * vision (multiCam + raspi software)
    * worldModel
    * motion control: motionPlanning, pathPlanning, velocityControl, ballHandling, shootPlanning
    * peripheralsInterface
    * rtdb (evolution on rtdb2 from Cambada repo) and related logging utils
    * visualizer
    * simulator
* all sorts of convenience scripting (a mix of python3 and bash)
* configuration files (typically .yaml)

It does not contain:
* teamplayData repo, which contains the tactics
* new motion platform repo
* data repo (incl. a few dependencies from code)

## Setup

Environment requirements:
* ubuntu20
* cmake
* several packages, see `package_list` and/or `wtf` script


