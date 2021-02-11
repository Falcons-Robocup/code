// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

