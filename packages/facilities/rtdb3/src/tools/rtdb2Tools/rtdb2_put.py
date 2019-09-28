""" 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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


# Main structure of the program
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("agent", help="the agent to write to (as int)", type=int)
    parser.add_argument("key", help="the key to put. E.g., ACTION")
    parser.add_argument("value", help="the value to put, as deserializable string")
    parser.add_argument("--path", help="the storage path where the RtDB2 databases are stored. Default: '{}'".format(RTDB2_DEFAULT_PATH), default=RTDB2_DEFAULT_PATH)
    args = parser.parse_args()

    # Create instance of RtDB2Store and read databases from disk
    rtdb2Store = RtDB2Store(args.path, False) # don't start in read-only
    rtdb2Store.refresh_rtdb_instances()

    # This put operation should try to preserve attributes, like shared
    value = eval(args.value) # yikes! but, this prevents argument like '[1, 4]' being written into database as string, whereas we need an array of size 2
    item = rtdb2Store.put(args.agent, args.key, value)

    rtdb2Store.closeAll()

