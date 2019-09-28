SmokeTest.robot

 Created on: Sep 16, 2019
     Author: Martijn van Veen
     Based on initial version of Coen Tempelaars
     Instead of playing for a fixed duration, play until the own team has scored
     with a time limit to avoid an endless test
     
*** Settings ***
Resource        ../resources/Simulator.robot
Test Teardown   Stop simulator

*** Test Cases ***
Smoke1
    Given Falcons simulator is started
    When Team "A" has scored at least once within "10" minutes
    Then Success
