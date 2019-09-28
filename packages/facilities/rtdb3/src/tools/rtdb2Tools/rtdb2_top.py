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

# Main structure of the program
if __name__ == "__main__":
    from rtdb2_curses import RtDBCurses
    from rtdb2 import RtDB2Store

    if len(sys.argv) != 2 and len(sys.argv) != 1:
        print "Expected the storage path."
        print "Usage: " + sys.argv[0] + " storage_path"
        sys.exit(0)

    if len(sys.argv) != 2:
        DEFAULT_PATH = "/tmp/rtdb2_storage"
        agents = os.listdir(DEFAULT_PATH)
        if len(agents) <= 0:
            print "No agents where found in %s" % (DEFAULT_PATH, )
            sys.exit(0)

        if len(agents) == 1:
            storage_path = os.path.join(DEFAULT_PATH, agents[0])
        else:
            while True:
                print "Found agents: %s" % (agents, )
                sys.stdout.write('Write agent number: ')

                agent = raw_input()
                agent = "agent" + agent
                if agent in agents:
                    storage_path = os.path.join(DEFAULT_PATH, agent)
                    break
    else:
        storage_path = sys.argv[1]

    rtdb2Store = RtDB2Store(storage_path)
    window = RtDBCurses()

    try:
        while True:
            info = rtdb2Store.getAllRtDBItems()
            window.display_info(info)
    except KeyboardInterrupt:
        pass
    finally:
        window.exit_screen()
        rtdb2Store.closeAll()
