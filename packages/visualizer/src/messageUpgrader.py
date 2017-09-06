""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
# 2016-07-16 JFEI creation
#
# Description: part of playbackMatchlog: message upgrade facility 
#
#
# 


import sys,os
from messageConverter import *



class MessageUpgrader():
    def __init__(self):
        pass

    def __call__(self, msgFrom, msgTypeStr):
        msgDict = convert_ros_message_to_dictionary(msgFrom)
        # try to simply convert
        try:
            msgTo = convert_dictionary_to_ros_message(msgTypeStr, msgDict)
        except:
            # try to upgrade
            try:
                msgTo = self.upgrade(msgFrom, msgTypeStr, msgDict)
            except:
                # maybe a new upgrader is needed?
                print "WARNING: could not convert message of type '%s'!" % (msgTypeStr)
                print 'given data as dict : ', msgDict
                msgNewClass = roslib.message.get_message_class(msgTypeStr)
                print 'target type as dict: ', convert_ros_message_to_dictionary(msgNewClass())
                raise
        return msgTo
        
    def upgrade(self, msgFrom, msgTypeStr, msgDict):
        # worldModel upgrader due to replacing single ball (NaN if empty) with array of balls (#405)
        if "worldmodel" in msgTypeStr:
            if isinstance(msgDict['ballpos'], dict):
                # check if output array should be empty
                if msgDict['ballpos']['x'] == float("nan"):
                    msgDict['ballpos'] = []
                else:
                    # wrap the ball in a list, remove phi component
                    ballMsg = msgDict['ballpos']
                    del ballMsg['phi']
                    msgDict['ballpos'] = [ballMsg]
        # new-teamplay has no actionHandler and reasoning anymore
        if "diag_active" in msgTypeStr:
            if "reasoning" in msgDict.keys():
                del msgDict['reasoning']
                del msgDict['actionHandler']
        # coach online state has changed in april 2017
        if "ana_online" in msgTypeStr:
            # clear 
            msgDict = {}
            msgDict['state'] = [0, 1, 1, 1, 1, 1, 1]
        # call converter again to capture an appropriate error
        msgTo = convert_dictionary_to_ros_message(msgTypeStr, msgDict)
        return msgTo


