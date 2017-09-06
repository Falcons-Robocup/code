 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /**
 *  CSLFltrs.h
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

#include "CSLFltrs.h"

/**
 * @fn CSL_Fltr_LowPass_1st
 * @brief Implementation of low pass filter
 *
 *            wlp
 * H(s) = k -------
 *          s + wlp
 *
 * @param const double k Gain of the filter
 * @param const double fp Low pass frequency
 */
CSLFltr_t CSL_Fltr_LowPass_1st(const double k, const double flp)
{
	CSLFltr_t fltr;

	// Change frequency to radial frequency
	double wlp = M_2PI*flp;

	// Set order of filter
	fltr.o = 1;

	// Allocate size for parameters
	fltr.a = malloc(1*sizeof(double));
	fltr.b = malloc(2*sizeof(double));

	// Set denominator and numerator
	fltr.a = (double[1]) {wlp};
	fltr.b = (double[2]) {k*wlp, 0};

	return fltr;
}

/**
 * @fn CSL_Fltr_HighPass_1st
 * @brief Implementation of high pass filter
 *
 *            s
 * H(s) = k -------
 *          s + whp
 *
 * @param const double k Gain of the filter
 * @param const double fhp High pass frequency
 */
CSLFltr_t CSL_Fltr_HighPass_1st(const double k, const double fhp)
{
	CSLFltr_t fltr;

	// Change frequency to radial frequency
	double whp = M_2PI*fhp;

	// Set order of filter
	fltr.o = 1;

	// Allocate size for parameters
	fltr.a = malloc(1*sizeof(double));
	fltr.b = malloc(2*sizeof(double));

	// Set denominator and numerator
	fltr.a = (double[1]) {whp};
	fltr.b = (double[2]) {0, k};

	return fltr;
}

/**
 * @fn CSL_Fltr_LeadLag
 * @brief Implementation of lead lag filter
 *
 *          wp s + wz
 * H(s) = K -- ------
 *          wz s + wp
 *
 * @param const double k Gain of the filter
 * @param const double fz Zero frequency
 * @param const double fp Pole frequency
 */
CSLFltr_t CSL_Fltr_LeadLag(const double k, const double fz, const double fp)
{
	CSLFltr_t fltr;

	// Change frequency to radial frequency
	double wz = M_2PI*fz;
	double wp = M_2PI*fp;

	// Set order of filter
	fltr.o = 1;

	// Allocate size for parameters
	fltr.a = malloc(1*sizeof(double));
	fltr.b = malloc(2*sizeof(double));

	// Set denominator and numerator
	fltr.a = (double[1]) {wp};
	fltr.b = (double[2]) {k*wp, k*wp/wz};

	return fltr;
}

/**
 * @fn CSL_Fltr_LowPass_2nd
 * @brief Implementation of low pass 2nd order filter
 *
 *                   wlp^2
 * H(s) = k -------------------------
 *          s^2 + 2*dlp*wlp*s + wlp^2
 *
 * @param const double k Gain of the filter
 * @param const double flp Low pass frequency
 * @param const double dlp Damping of pole
 */
CSLFltr_t CSL_Fltr_LowPass_2nd(const double k, const double flp,
	const double dlp)
{
	CSLFltr_t fltr;

	// Change frequency to radial frequency
	double wlp = M_2PI*flp;

	// Set order of filter
	fltr.o = 1;

	// Allocate size for parameters
	fltr.a = malloc(1*sizeof(double));
	fltr.b = malloc(2*sizeof(double));

	// Set denominator and numerator
	fltr.a = (double[2]) {wlp*wlp, 2*dlp*wlp};
	fltr.b = (double[3]) {k*wlp*wlp, 0, 0};

	return fltr;
}

/**
 * @fn CSL_Fltr_HighPass_2nd
 * @brief Implementation of high pass filter
 *
 *                    s^2
 * H(s) = k -------------------------
 *          s^2 + 2*dhp*whp*s + whp^2
 *
 * @param const double g Gain of the filter
 * @param const double fhp High pass frequency
 * @param const double dhp High pass damping
 */
CSLFltr_t CSL_Fltr_HighPass_2nd(const double k, const double fhp,
	const double dhp)
{
	CSLFltr_t fltr;

	// Change frequency to radial frequency
	double whp = M_2PI*fhp;

	// Set order of filter
	fltr.o = 2;

	// Allocate size for parameters
	fltr.a = malloc(2*sizeof(double));
	fltr.b = malloc(3*sizeof(double));

	// Set denominator and numerator
	fltr.a = (double[2]) {whp*whp, 2*dhp*whp};
	fltr.b = (double[3]) {0, 0, k};

	return fltr;
}

/**
 * @fn CSL_Fltr_Bandpass
 * @brief Implementation of band pass filter
 *
 *                     wlp*s
 * H(s) = k -----------------------------
 *          s^2 + (wlp + whp)*s + wlp*whp
 *
 * @param const double g Gain of the filter
 * @param const double flp Low pass frequency
 * @param const double fhp High pass frequency
 */
CSLFltr_t CSL_Fltr_BandPass(const double k, const double flp, const double fhp)
{
	CSLFltr_t fltr;

	// Change frequency to radial frequency
	double wlp = M_2PI*flp;
	double whp = M_2PI*fhp;

	// Set order of filter
	fltr.o = 2;

	// Allocate size for parameters
	fltr.a = malloc(2*sizeof(double));
	fltr.b = malloc(3*sizeof(double));

	// Set denominator and numerator
	fltr.a = (double[2]) {wlp*whp, (wlp + whp)};
	fltr.b = (double[3]) {0, k*wlp, 0};

	return fltr;
}

/**
 * @fn CSL_Fltr_Notch
 * @brief Implementation of notch filter
 *
 *          wp s^2 + 2*dz*wz*s + wz^2
 * H(s) = K -- ----------------------
 *          wz s^2 + 2*dp*wp*s + wp^2
 *
 * @param const double k Gain of the filter
 * @param const double fz Zero frequency
 * @param const double dz Damping of zero
 * @param const double fp Pole of frequency
 * @param const double dp Damping of pole
 */
CSLFltr_t CSL_Fltr_Notch(const double k, const double fz, const double dz,
    const double fp, const double dp)
{
	CSLFltr_t fltr;

	// Change frequency to radial frequency
	double wz = M_2PI*fz;
	double wp = M_2PI*fp;

	// Set order of filter
	fltr.o = 2;

	// Allocate size for parameters
	fltr.a = malloc(2*sizeof(double));
	fltr.b = malloc(3*sizeof(double));

	// Set denominator and numerator
	fltr.a = (double[2]) {wp*wp, 2*dp*wp};
	fltr.b = (double[3]) {k*wp*wp, k*wp*wp*2*dz/wz, k*wp*wp/(wz*wz)};

	return fltr;
}

/**
 * @fn Fltr_Discretize_Tustin
 * @brief Discretize using the billinear method without pre-warping frequency
 * @param CSLFltr_t *fltr The filter which needs to be discretized
 */
void CSL_Fltr_Discretize_Tustin(CSLFltr_t *fltr, const double fs)
{
	const double Ts  = 1/fs;
	const double Ts2 = Ts*Ts;
	double *a, *b, divisor;

	a = malloc( fltr->o   *sizeof(double));
	b = malloc((fltr->o+1)*sizeof(double));

	if (fltr->o == 1) {
		divisor = Ts*fltr->a[0] + 2;

		a = (double[1]) {(Ts*fltr->a[0] - 2) / divisor};

		b = (double[2]) {(Ts*fltr->b[0] - 2*fltr->b[1]) / divisor, \
						 (Ts*fltr->b[0] + 2*fltr->b[1]) / divisor};
	} else if (fltr->o == 2) {const
		divisor = Ts2*fltr->a[0] + 2*Ts*fltr->a[1] + 4;

		a = (double[2]) {(  Ts2*fltr->a[0] - 2*Ts*fltr->a[1] + 4) / divisor, \
			 	 	 	 (2*Ts2*fltr->a[0]                   - 8) / divisor};

		b = (double[3]) {(  Ts2*fltr->b[0] - 2*Ts*fltr->b[1] + 4*fltr->b[2]) / divisor, \
			 	 	 	 (2*Ts2*fltr->b[0]                   - 8*fltr->b[2]) / divisor,	\
			 	 	 	 (  Ts2*fltr->b[0] + 2*Ts*fltr->b[1] + 4*fltr->b[2]) / divisor};
	}

	free(fltr->a);
	free(fltr->b);

	memcpy(fltr->a,a, fltr->o     *sizeof(double));
	memcpy(fltr->b,b,(fltr->o + 1)*sizeof(double));
}

/**
 * @fn Fltr_Discretize_Tustin_Warp
 * @brief Discretize using the billinear method with pre-warping frequency
 * @param CSLFltr_t *fltr The filter which needs to be discretized
 * @param double f0 Tustins prewarp frequency, set zero if not used
 */
void CSL_Fltr_Discretize_Tustin_Warp(CSLFltr_t *fltr, const double fs,
	const double f0)
{
	const double Ts  = 1/fs;
	const double w0  = M_2PI*f0;
	const double w02 = w0*w0;
	const double th  = tan(Ts*w0/(2*fs));
	const double th2 = th*th;
	double *a, *b, divisor;

	a = malloc( fltr->o   *sizeof(double));
	b = malloc((fltr->o+1)*sizeof(double));

	if (fltr->o == 1) {
		divisor = th*fltr->a[0] + w0;

		a = (double[1]) {(th*fltr->a[0] - w0) / divisor};

		b = (double[2]) {(th*fltr->b[0] - w0*fltr->b[1]) / divisor, \
			 	 	 	 (th*fltr->b[0] + w0*fltr->b[1]) / divisor};
	} else if (fltr->o == 2) {
		divisor = th2*fltr->a[0] + w0*th*fltr->a[1] + w02;

		a = (double[2]) {(  th2*fltr->a[0] - w0*th*fltr->a[1] +   w02) / divisor, \
			 	 	 	 (2*th2*fltr->a[0]                    - 2*w02) / divisor};

		b = (double[3]) {(  th2*fltr->b[0] - w0*th*fltr->b[1] +   w02*fltr->b[2]) / divisor, \
						 (2*th2*fltr->b[0]                    - 2*w02*fltr->b[2]) / divisor, \
						 (  th2*fltr->b[0] + w0*th*fltr->b[1] +   w02*fltr->b[2]) / divisor};
	}

	free(fltr->a);
	free(fltr->b);

	memcpy(fltr->a,a, fltr->o     *sizeof(double));
	memcpy(fltr->b,b,(fltr->o + 1)*sizeof(double));
}
