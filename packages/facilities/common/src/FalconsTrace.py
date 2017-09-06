""" 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/env python
# JFEI 20150530 created python tracing similar to the C++ functionality
# usage: 
#   from FalconsTrace import trace
#   # anywhere in your code
#   trace("somestring with formatargs", 5, "some argument or variable")
#
# TODO: consider replacing this with 'logging' builtin module
#

import os, sys
import traceback
import datetime
import FalconsEnv
import threading
import subprocess


_trace_file_obj = None
_trace_file_name = None
_trace_file_name_override = None
_trace_file_mode = "w"
_trace_logdir = None
_tracing_enabled = False
_last_check = datetime.datetime.now() - datetime.timedelta(1, 1)
TRIGGERFILE = '/var/tmp/tracing_trigger'
_lock = threading.Lock()


def _update():
    """Inspect tracing trigger file and open trace file if needed."""
    global _last_check, _tracing_enabled, _trace_logdir, _trace_file_obj, _trace_file_name
    time_now = datetime.datetime.now()
    elapsed = (time_now - _last_check).total_seconds()
    if elapsed > 1:
        _last_check = time_now
        if os.path.isfile(TRIGGERFILE):
            # open if not already done
            if not _tracing_enabled:
                # determine directory
                _trace_logdir = '/var/tmp'
                lines = open(TRIGGERFILE).readlines()
                if len(lines) and os.path.isdir(lines[0].strip()):
                    _trace_logdir = lines[0].strip()
                # determine filename
                if _trace_file_name_override == None:
                    processId = subprocess.check_output("getProcessId", shell=True).strip()
                    _trace_file_name = "%s/trace_%c%d_%s.txt" % (_trace_logdir, FalconsEnv.get_team_name()[-1], FalconsEnv.get_robot_num(), processId)
                    counter = 1
                    while os.path.isfile(_trace_file_name):
                        counter += 1
                        _trace_file_name = "%s/trace_%c%d_%s_%d.txt" % (_trace_logdir, FalconsEnv.get_team_name()[-1], FalconsEnv.get_robot_num(), processId, counter)
                else:
                    _trace_file_name = "%s/%s" % (_trace_logdir, _trace_file_name_override)
                _trace_file_obj = open(_trace_file_name, _trace_file_mode, 1)
            _tracing_enabled = True
        else:
            # close if tracing was enabled
            if _tracing_enabled:
                _trace_file_obj.close()
            _tracing_enabled = False
                
        
def trace(*args):
    with _lock:
        _update()
    timestamp = (str(datetime.datetime.now()).replace(' ', ','))
    stack = traceback.extract_stack()
    frame = stack[-2]
    filename = frame[0]
    linenum = frame[1]
    funcname = frame[2]
    # try formatting
    if "%" in args[0]:
       args_formatted = args[0] % args[1:]
    else:
       # just convert all to string
       args_formatted = str(args[0])
    message = "%s %s %s:%04d %s %s\n" % (timestamp, threading.current_thread(), filename, linenum, funcname, args_formatted)
    #print "debugmsg", message
    if not _tracing_enabled:
        return
    with _lock:
        _trace_file_obj.write(message)
    # JFEI stack inspection
    #traceback.print_stack(file=_trace_file_obj)
    return message
    
def traceError(*args):
    '''
    Traces message in default tracing when enabled
    Prints same message in stdErr as well    
    '''
    errorMessage = trace(args)
    sys.stderr.write(errorMessage)
