 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef CONFIGPERIPHERALSINTERFACEBALLHANDLERS_HPP_
#define CONFIGPERIPHERALSINTERFACEBALLHANDLERS_HPP_

#include "RtDB2.h" // required for serialization


struct ConfigPeripheralsInterfaceBallHandlers
{

	bool ballhandlerPlotEnabled = false; 	//Enable Data Dump for plotting
	int Kp = 1500; 							//BallHandler P-value of PID (max=65536)
	int Ki = 0; 							//BallHandler I-value of PID (max=65536)
	int Kd = 0; 							//BallHandler D-value of PID (max=65536)
	int iTh = 0; 							//Threshold of using I (iMin) -- (max=65536)
	int iMax = 4000; 						//Limits the maximum I value (max=100000)
	int Ang_Kp = 8000; 						//BallHandler Angle P-value of PID (max=65536)
	int Ang_Ki = 0; 						//BallHandler Angle I-value of PID (max=65536)
	int Ang_Kd = 0; 						//BallHandler Angle D-value of PID (max=65536)
	int PwmMax = 648; 						//PWM Maximum power (max=648)
	int PwmMaxDeltaSize = 65536; 			//PWM maximum step size per cycle (2.5ms, 400Hz, max=648<<16)

    SERIALIZE_DATA(ballhandlerPlotEnabled, Kp, Ki, Kd, iTh, iMax, Ang_Kp, Ang_Ki, Ang_Kd, PwmMax, PwmMaxDeltaSize);
};

#endif

