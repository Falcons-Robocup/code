# Copyright 2016 martijn van veen (Falcons)
# SPDX-License-Identifier: Apache-2.0
# Generates State_Manager_Transition_Table.yaml based on State_Manager_Transition_Table.csv
# By Martijn van Veen
# Version 1.1 - Tabs not allowed in YAML, replace with spaces
# Version 1.2 - Now outputs valid YAML
# Last edited February 2 2016

out = open("../../../../config/State_Manager_Transition_Table.yaml","w")

# write yaml header
out.write("---\n")

# Open input table; first row are the ref box signals used for state transition
f = open("State_Manager_Transition_Table.csv")
inputs = []
line = f.readline()
inputs = line.split(",")

# for each game state (row) determine the next game state based on ref box signal 
for line in f.readlines() :
        line_items = line.split(",")
        out.write("- GameState: " + line_items[0].strip() +"\n")
        for index in range(len(inputs)-1):
                out.write("- - RefBoxSignal: "+ inputs[index+1].strip()+ "\n")
                out.write("  - - NewGameState: "+line_items[index+1].strip() + "\n")

out.write("...\n")
out.close()
f.close()
