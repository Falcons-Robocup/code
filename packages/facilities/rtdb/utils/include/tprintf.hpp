 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /**
 * File: tprintf.hpp
 * Author: Jan Feitsma
 * Creation: January 2019
 *
 * Description: wrapper around printf, pre-pending timestamp and always flushing.
 * Having the timestamp is convenient to find when something interesting occured. 
 * Consider for instance a fragment of standard output of comm process without timestamps:
 *   transmitting 33.8KB/s (averaged over 1.0s)
 *   transmitting 34.5KB/s (averaged over 1.0s)
 *   transmitting 73.8KB/s (averaged over 1.0s)
 *   transmitting 33.8KB/s (averaged over 1.0s)
 *   transmitting 33.8KB/s (averaged over 1.0s)
 * When did the 73KB/s spike occur?
 *
 */

#ifndef _INCLUDED_TPRINTF_HPP_
#define _INCLUDED_TPRINTF_HPP_

#include <cstdarg>

// uncomment to suppress all output
//#define TPRINTF_DISABLED

// helpers
void _tprintf_start();
void _tprintf_end();

// tprintf macro
#ifndef TPRINTF_DISABLED
    #define tprintf(fmt, ...) { _tprintf_start(); printf(fmt, ##__VA_ARGS__); _tprintf_end(); }
#else
    #define tprintf(...) ((void)0)
#endif

#endif

