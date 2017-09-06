 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnosticsDutyCycle.hpp
 *
 *  Created on: Mar 07, 2017
 *      Author: Jan Feitsma
 */

#ifndef CDIAGNOSTICSDUTYCYCLE_HPP_
#define CDIAGNOSTICSDUTYCYCLE_HPP_


#include <vector>
#include <string>


namespace diagnostics
{

class cDiagnosticsDutyCycle
{
  public:
    cDiagnosticsDutyCycle(std::string name, int freq, float warningThreshold=0.3);
    ~cDiagnosticsDutyCycle();
    void pokeStart();
    void pokeEnd();
    
  private:
    std::string       _name;             // name of process to measure
    int               _frequency;        // expected frequency (typically heartbeat)
    float             _warningThreshold; // level at which warnings are generated (default 30%)
    bool              _active;           // internal flag to check if client process is calling us properly
    double            _lastStart;        // timestamp stored at pokeStart
    
    // helper function
    void handleElapsed(float elapsed);
};

} // end of namespace diagnostics

#endif /* CDIAGNOSTICSDUTYCYCLE_HPP_ */

