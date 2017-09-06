 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /**
 *  CSLCtrlsImpl.c
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

#include <math.h>
#include "CSLCtrlsImpl.h"

/**
 * @fn CSL_Ctrl_PID_Create
 * @brief Create PID controller
 * @param CSLPIDState_t *PID Holds PID controller object
 */
CSLPIDState_t Ctrl_PID_Create(CSLPID_t PID)
{
	CSLPIDState_t state;

	// Allocate size
	state.c = malloc(6*sizeof(double));
	state.n = malloc(4*sizeof(double));

	// Set coefficients
	memcpy(state.c,    PID.CtrlPDLP.a,2*sizeof(double));
	memcpy(state.c + 2,PID.CtrlPDLP.b,3*sizeof(double));
	memcpy(state.c + 5,PID.CtrlI.a,   1*sizeof(double));
	memcpy(state.c + 6,PID.CtrlI.b,   2*sizeof(double));

	// Set internal state
	state.n = (double[4]) {0, 0, 0, 0};

	// Set saturations and thresholds
	state.satP   = PID.satP;
	state.satI   = PID.satI;
	state.satPID = PID.satPID;
	state.thrI   = PID.thrI;

	return state;
}

/**
 * @fn CSL_Ctrl_Calculate_PID
 * @brief Implements calculation method for DFII PID controller
 * Number of delays: 2
 * Number of additions: 4
 * Number of multiplications: 5
 *
 *                .---.                             .-----.                 .---.    .-----.
 * x[k] ---+----->| + |-------------------+-------->|  b2 |---------------->| + |--->| sat |---> y[k]
 * 		   |      '---'                   | n[k]    '-----'                 '---'    '-----'
 *         |        ^                     v                                   ^
 *         |        |                  .-----.                                |
 *         |        |                  | z-1 |                                |
 * 		   |        |                  '-----'                                |
 *         |      .---.     .-----.       | n[k-1]  .-----.                 .---.
 *         |      | + |<----| -a1 |<------+-------->|  b1 |---------------->| + |
 *         |      '---'     '-----'       |         '-----'                 '---'
 *         |        ^                     v                                   ^
 *         |        |                  .-----.                                |
 *         |        |	               | z-1 |                                |
 * 		   v        |                  '-----'                                |
 * 	   .-------.    |       .-----.       | n[k-2]  .-----.     .-----.     .---.
 * 	   | thres |    +-------| -a0 |<------+-------->|  b0 |---->| Sat |---->| + |
 * 	   | -hold |            '-----'                 '-----'     '-----'     '---'
 *     '-------'                                                              ^
 *         |                                                                  |
 *         |                .-----.     .---.       .-----.     .-----.       |
 *         +--------------->|  a2 |---->| + |------>| sat |---->| z-1 |-------+
 *                          '-----'     '---'       '-----'     '-----'       |
 *                                        ^                            m[k-1] |
 *                                        |                                   |
 *                                        +-----------------------------------+
 *
 * @param const double input Input of the filter
 * @param CSLPIDState_t *state Holds the state of the filter
 * @param double *output Output of the filter
 */
void CSL_Ctrl_Calculate_PID(const double input, CSLPIDState_t *state, double *output)
{
	// c[0] = a0, c[1] = a1, c[2] = b0, c[3] = b1, c[4] = b2, c[5] = a2
	// n[0] = n[k-2], n[1] = n[k-1], n[2] = n[k], n[3] = m[k-1]

	// Set intermediate state
	state->n[2] = - state->n[0]*state->c[0] \
			      - state->n[1]*state->c[1] \
			      + input;

	// Check if threshold is met
	if (input < state->thrI.lower || state->thrI.upper < input) {
		// Set intermediate state
		state->n[3] = CSL_Saturation_Asym(state->c[5]*input + state->n[3],
			                              state->satI);
	}

	// Output of PD controller, low pass filter and I controller
	output = Saturation_Asym(
			     Saturation_Asym(state->n[0]*state->c[2],state->satP)
				 + state->n[1]*state->c[3]
				 + state->n[2]*state->c[4]
				 + state->n[3],state->satPID);

	// Update states
	state->n[0] = state->n[1];
	state->n[1] = state->n[2];
}
