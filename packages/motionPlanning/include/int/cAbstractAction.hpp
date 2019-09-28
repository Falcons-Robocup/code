 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cAbstractAction.hpp
 *
 *  Created on: Nov 17, 2017
 *      Author: Jan Feitsma
 */

#ifndef CABSTRACTACTION_HPP_
#define CABSTRACTACTION_HPP_

#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>


#include "int/adapters/cRTDBOutputAdapter.hpp"

#include "int/cInterfaces.hpp"
#include "int/cTimer.hpp"
#include "tracing.hpp"
#include "FalconsRtDB2.hpp" // actionResult

class cAbstractAction
{
public:
    cAbstractAction();
    virtual ~cAbstractAction();
    
    void setParameters(std::vector<std::string> const &params);
    virtual actionResultTypeEnum execute() = 0;
    void connect(cInterfaces *interfaces = NULL);
    float elapsed(); // time in seconds
    
    // shared convenience functions
    void stopMoving();

protected:
    std::vector<std::string> _params;
    bool _isConnected;
    
    cWorldModelInterface *_wm;
    cRTDBOutputAdapter *_rtdbOutput;

    cTimer _actionTimer;
};

#endif /* CABSTRACTACTION_HPP_ */

