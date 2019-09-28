 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * testIMU.cpp
 *
 *  Created on: July 7, 2018
 *      Author: Jan Feitsma
 */

#include "int/IMU/IMU.hpp"
#include "FalconsRtDB2.hpp"

// MAIN
int main(int argc, char **argv)
{
    auto imu = IMU();
    double delta = 0.0;
    double prev = 0.0;
    double t0 = rtime::now();
    
    while (true)
    {
        //Vector3 v = imu.getVector();
        double t = rtime::now();
        float angle = imu.getCompassAngle();
        
        if (angle != prev)
        {
            prev = angle;
            delta = t - t0;
            t0 = t;
        }
        
        //printf("%10.2f %10.2f %10.2f %10.2f %10.2f\n", angle, v.x(), v.y(), v.z(), delta);
        printf("%16.6f %10.2f %10.2f\n", t, angle, delta);
        fflush(stdout);
        
        usleep(10000);
    }
    
    
    return 0;
}

