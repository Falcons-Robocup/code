 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /**
 *  CSLCtrlsImpl.h
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

/**
 * @struct CSLPIDState_t
 * @param double *c Holds the coefficients of the filter
 * @param double *n Condition of intermediate states
 */
typedef struct CSLPIDState {
	double *c;
	double *n;
	CSLIntval_t satP;
	CSLIntval_t satI;
	CSLIntval_t satPID;
	CSLIntval_t thrI;
} CSLPIDState_t;

/**
 * @fn CSL_Ctrl_PID_Create
 * @brief Create PID controller
 * @param PIDState_t *PID Holds PID controller object
 */
CSLPIDState_t CSL_Ctrl_PID_Create(CSLPID_t PID);

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
 * @param PIDState_t *state Holds the state of the filter
 * @param double *output Output of the filter
 */
void CSL_Ctrl_Calculate_PID(const double input, CSLPIDState_t *state, double *output);
