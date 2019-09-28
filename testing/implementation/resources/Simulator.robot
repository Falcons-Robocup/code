Simulator.robot

 Created on: Aug 29, 2019
     Author: Coen Tempelaars


*** Settings ***
Library         BuiltIn
Library         SimulatorLibrary

*** Keywords ***
Given Falcons simulator is started
    Start simulator

When "${s}" seconds have passed
    Sleep    ${s}

When "${m}" minutes have passed
    Sleep    ${m} minutes

Then Team "${t}" has scored at least once
    Minimum score    ${t}    1

When Team "${t}" has scored at least once within "${m}" minutes
    Wait Until Keyword Succeeds  ${m} minutes  30 seconds  Minimum score    ${t}    1
    
Then Success
    Success
        

