 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * test_rtime.cpp
 *
 * Tester for timestamp utilities, conversions, standards.
 *
 */


#include "../rtime.hpp"


int main()
{
    // test now() and string conversion
    rtime t = rtime::now();
    printf("now() as string: %s\n", t.toStr().c_str());
    
    // double conversion
    double d = (double)t;
    printf("now() as double: %.6f\n", d);
    
    // float conversion should generate a compile-time error
    // reason: precision would be lost, which is typically not desired
    // and runtime errors can be too obscure to trigger / pinpoint
    //float f = (float)t;
    
    // operator-
    rtime delta = t - t;
    printf("delta as string: %s\n", delta.toStr().c_str()); // unix EPOCH (1-1-1970), possibly with some hours timezone offset
    printf("delta as double: %.6f\n", double(delta));
    
    // operator+
    rtime tnext = t + 0.033;
    printf("tnext as string: %s\n", tnext.toStr().c_str());
    printf("tnext as double: %.6f\n", double(tnext));
    
    // convertors
    rtime t2;
    t2.fromDouble(d);
    printf("equality after conversion from double: %d\n", (t2 == t));
    
    // delta after subtraction
    printf("dt after operator- as double: %.6f\n", double(tnext - t)); // should show 0.033
    printf("dt after operator- as float: %.6f (requires explicit cast)\n", float(double(tnext - t)));
    
    exit(0);
}


