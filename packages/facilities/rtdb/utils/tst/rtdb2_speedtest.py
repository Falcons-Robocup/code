""" 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 """ 
 #! /usr/bin/env python3

import falconspy

from rtdb2 import RtDB2Store, RTDB2_DEFAULT_PATH

ITERATIONS = 10000

if __name__ == '__main__':
    print "Sending {} samples to RtDB...".format(ITERATIONS)

    rtdb2Store = RtDB2Store(RTDB2_DEFAULT_PATH, False)
    rtdb2Store.refresh_rtdb_instances()

    ages = list()
    for i in range(ITERATIONS):
        rtdb2Store.rtdb_instances["agent_shared0"].put("TEST", i)
        (value, age) = rtdb2Store.rtdb_instances["agent_shared0"].get("TEST")
        ages.append(age)

    print "Got the following ages (lifes) :"
    print "Min: {} [ms]".format(min(ages))
    print "Max: {} [ms]".format(max(ages))
    print "Avg: {} [ms]".format(sum(ages)/len(ages))
