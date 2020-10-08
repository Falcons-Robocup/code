""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #!/usr/bin/python
import argparse
import rtdb2tools


# Main structure of the program
if __name__ == "__main__":

    # Argument parsing.
    descriptionTxt = 'This tool monitors a RtDB item, continuously displaying its value until CTRL-C is pressed.\n'
    exampleTxt = 'Example: rtdb2_mon.py -a 4 ACTION\n'
    parser     = argparse.ArgumentParser(description=descriptionTxt, epilog=exampleTxt,  formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-a', '--agent', help='agent ID to use', type=int, default=rtdb2tools.guessAgentId())
    parser.add_argument('-f', '--frequency', help='refresh frequency in Hz', type=float, default=10)
    parser.add_argument('-t', '--timestamp', help='prepend timestamp', action='store_true')
    #parser.add_argument('-c', '--onchange', help='show items directly when changed, minimizing latency', action='store_true')
    # TODO: zero-latency '--onchange' option requires RTDB wait_for_put, currently not implemented
    parser.add_argument('-s', '--showonce', help='filter duplicates, show stale items once', action='store_true')
    parser.add_argument('-p', '--path', help='database path to use', type=str, default=rtdb2tools.RTDB2_DEFAULT_PATH)
    parser.add_argument('key', help='RtDB key to read')
    args = parser.parse_args()

    # Instantiate the monitor
    r = rtdb2tools.RTDBMonitor(args.agent, args.frequency, args.path)
    r.subscribe(args.key)
    r.prependTimestamp = args.timestamp
    r.showOnce = args.showonce

    # Run
    r.run()

