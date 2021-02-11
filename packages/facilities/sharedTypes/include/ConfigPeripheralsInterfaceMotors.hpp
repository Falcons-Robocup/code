// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGPERIPHERALSINTERFACEMOTORS_HPP_
#define CONFIGPERIPHERALSINTERFACEMOTORS_HPP_

#include "RtDB2.h" // required for serialization


struct ConfigPeripheralsInterfaceMotors
{

	bool motorPlotEnabled = false; 		//Enable Data Dump for plotting
	int Kp = 2560; 						//P-value of PID (max=65536)
	int Ki = 768; 						//I-value of PID (max=65536)
	int Kd = 0; 						//D-value of PID (max=65536)
	int iTh = 512; 						//Threshold of using I (iMin) -- (max=65536)
	int iMax = 4000; 					//Limits the maximum I value (max=100000)
	int PwmMax = 648; 					//PWM Maximum power (max=648)
	int PwmMaxDeltaSize = 65536; 		//PWM maximum step size per cycle (2.5ms, 400Hz, max=648<<16)

    SERIALIZE_DATA(motorPlotEnabled, Kp, Ki, Kd, iTh, iMax, PwmMax, PwmMaxDeltaSize);
};

#endif

