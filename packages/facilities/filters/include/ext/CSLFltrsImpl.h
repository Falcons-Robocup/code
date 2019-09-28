 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /**
 *  CSLFltrsImpl.h
 *
 *  Author:           Wouter Geelen
 *  Contact:          wouter.geelen@gmail.com
 *  Created on:       2016-03-02
 *  Last modified on: 2016-10-22
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

#ifndef CSL_FLTR_IMPL_H_
#define CSL_FLTR_IMPL_H_

#include "CSLFltrs.h"

/**
 * @struct CSLFltrState_t
 * @brief Data structure which holds the state of the filter
 * @param double *c Holds the coefficients of the filter
 * @param double *n Intermediate state values
 */
typedef struct CSLFltrState {
	double *c;
	double *n;
} CSLFltrState_t;

/**
 * @fn CSL_Fltr_Create
 * @brief Create 1st or 2nd order filter by filter object
 * @param Fltr_t *fltr Holds filter object
 */
CSLFltrState_t CSL_Fltr_Create(CSLFltr_t fltr);

/**
 * @fn CSL_Fltr_Calculate_1st
 *
 * @brief Implements calculation method for DFII 1st order filter
 *
 *        b1*z + b0
 * H(z) = ---------
 *         z + a0
 *
 * Number of delays: 2
 * Number of additions: 4
 * Number of multiplications: 5
 *
 *          .---.                           .-----.     .---.
 * x[k] --->| + |-----------------+-------->|  b0 |---->| + |----> y[k]
 * 		    '---'                 | n[k]    '-----'     '---'
 *            ^                   v                       ^
 *            |                .-----.                    |
 *            |                | z-1 |                    |
 * 		      |                '-----'                    |
 *            |       .-----.     | n[k-1]  .-----.       |
 *            +-------| -a0 |<----+-------->|  b0 |-------+
 *                    '-----'               '-----'
 *
 * @param const double input Input of the filter
 * @param CSLFltrState_t *state Holds the state of the filter
 * @param double *output Output of the filter
 */
void CSL_Fltr_Calculate_1st(const double input, CSLFltrState_t *state,
	double *output);

/**
 * @fn CSL_Fltr_Calculate_2nd
 *
 * @brief Implements calculation method for DFII 2nd order filter
 *
 *        b2*z^2 + b1*z + b0
 * H(z) = ------------------
 *         z^2 + a1*z + a0
 *
 * Number of delays: 2
 * Number of additions: 4
 * Number of multiplications: 5
 *
 *          .---.                           .-----.     .---.
 * x[k] --->| + |-----------------+-------->|  b2 |---->| + |----> y[k]
 * 		    '---'                 | n[k]    '-----'     '---'
 *            ^                   v                       ^
 *            |                .-----.                    |
 *            |                | z-1 |                    |
 * 		      |                '-----'                    |
 *          .---.     .-----.     | n[k-1]  .-----.     .---.
 *          | + |<----| -a1 |<----+-------->|  b1 |---->| + |
 *          '---'     '-----'     |         '-----'     '---'
 *            ^                   v                       ^
 *            |                .-----.                    |
 *            |	               | z-1 |                    |
 * 		      |                '-----'                    |
 * 		      |       .-----.     | n[k-2]  .-----.       |
 * 		      +-------| -a0 |<----+-------->|  b0 |-------+
 * 				      '-----'               '-----'
 *
 * @param const double input Input of the filter
 * @param CSLFltrState_t *state Holds the state of the filter
 * @param double *output Output of the filter
 */
void CSL_Fltr_Calculate_2nd(const double input, CSLFltrState_t *state,
	double *output);

#endif /* CSL_FLTR_IMPL_H_ */
