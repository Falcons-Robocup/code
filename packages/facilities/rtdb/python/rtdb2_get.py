""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python
import os
import sys
import argparse
from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH
import rtdb2tools
from hexdump import hexdump


# Main structure of the program
if __name__ == "__main__":

    # Argument parsing.
    descriptionTxt = 'This tool reads a value from the database given an RtDB key.\n'
    exampleTxt = """Example: rtdb2_get.py -a 6 ROBOT_STATE
   age: 2h
shared: True
  list: False
 value: [2, [1581172987, 618438], [0.05368572473526001, -0.2938263416290283, 5.330356597900391], [0.1385340541601181, -0.8020891547203064, 0.7817431688308716], False, [0.0, 0.0], 6, 'A']
"""
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt,  formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('-a', '--agent', help='agent ID to use', type=int, default=rtdb2tools.guessAgentId())
    parser.add_argument('-s', '--serialized', help='also show serialized string (as hexdump)', action='store_true')
    parser.add_argument('-p', '--path', help='database path to use', type=str, default=RTDB2_DEFAULT_PATH)
    parser.add_argument('key', help='RtDB key to read')
    args = parser.parse_args()

    # Create instance of RtDB2Store and read databases from disk
    rtdb2Store = RtDB2Store(args.path)

    item = rtdb2Store.get(args.agent, args.key, timeout=None)
    print(str(item))
    if args.serialized:
        hexdump(item.value_serialized)

    rtdb2Store.closeAll()

