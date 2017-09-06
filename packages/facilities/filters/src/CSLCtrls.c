 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /**
 *  CSLCtrls.h
 *
 *  Author:           Wouter Geelen
 *  Contact:          wouter.geelen@gmail.com
 *  Created on:       2016-03-02
 *  Last modified on: 2016-03-02
 *
 *  Copyright [2016] [Wouter Geelen]
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *	you may not use this file except in compliance with the License.
 *	You may obtain a copy of the License at
 *
 *		http://www.apache.org/licenses/LICENSE-2.0
 *
 *	Unless required by applicable law or agreed to in writing, software
 *	distributed under the License is distributed on an "AS IS" BASIS,
 *	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *	See the License for the specific language governing permissions and
 *	limitations under the License.
 */

#include "CSLCtrls.h"

/**
 * @fn CSL_Ctrl_PID
 * @brief Implementation of PID controller
 *
 *          wlp^2*(s/wd + 1 + wi/s)
 * H(s) = k -----------------------
 *          s + 2*dlp*wlp*s + wlp^2
 *
 *        wlp^2*(kd*s + kp + ki/s)
 *      = -----------------------
 *		  s + 2*dlp*wlp*s + wlp^2
 *
 *		  Ki   (wlp^2*kp/wd - kp*wi)*s + (wlp^2 - 2*blp*wlp*kp*wi)
 *	    = -- + ---------------------------------------------------
 *	       s              s^2 + 2*blp*wlp*s + wlp^2
 *
 * @param const double k Proportional gain
 * @param const double fi Integrator frequency
 * @param const double fd Derivative frequency
 * @param const double flp Low pass filter frequency
 * @param const double dlp Damping of low pass filter
 * @param const CSLIntval_t satP Saturation level of P action
 * @param const CSLIntval_t satI Saturation level of I action
 * @param const CSLIntval_t satPID Saturation level of PID
 * @param const CSLIntval_t thrI Threshold for I action
 */
CSLPID_t CSL_Ctrl_PID(const double k, const double fi, const double fd,
	const double flp, const double dlp, const CSLIntval_t satP,
	const CSLIntval_t satI, const CSLIntval_t satPID, const CSLIntval_t thrI)
{
	CSLFltr_t CtrlPDLP;
	CSLFltr_t CtrlI;

	// Define radians
	double wi  = M_2PI*fi;
	double wd  = M_2PI*fd;
	double wlp = M_2PI*flp;

	// Define PD controller & lowpass filter
	CtrlPDLP.o = 2;
	CtrlPDLP.a = malloc(2*sizeof(double));
	CtrlPDLP.b = malloc(3*sizeof(double));

	// Set continuous-time parameters denominator
	CtrlPDLP.a = (double[2]) {wlp*wlp,2*dlp*wlp};
	// Set continuous-time parameters numerator
	CtrlPDLP.b = (double[3]) {wlp*wlp - 2*dlp*wlp*k*wi,wlp*wlp*k/wd - k*wi,0};

	// Define I controller
	CtrlI.o = 1;
	CtrlI.a = malloc(1*sizeof(double));
	CtrlI.b = malloc(2*sizeof(double));

	// Set continuous-time parameters denominator
	CtrlI.a = (double[1]) {0};
	// Set continuous-time parameters numerator
	CtrlI.b = (double[2]) {k*wi};

	// Setup PID controller
	CSLPID_t PID;
	PID.CtrlPDLP = CtrlPDLP;
	PID.CtrlI    = CtrlI;
	PID.satP     = satP;
	PID.satI     = satI;
	PID.satPID   = satPID;
	PID.thrI     = thrI;

	return PID;
}
