""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
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
