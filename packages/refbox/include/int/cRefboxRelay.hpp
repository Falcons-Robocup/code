 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRefboxRelay.hpp
 *
 *  Created on: Oct 20, 2010
 *      Author: Jan Feitsma
 */

#ifndef CREFBOXRELAY_HPP_
#define CREFBOXRELAY_HPP_


#include <string>
#include "int/TCPIP_client.h"
#include "int/RTDBAdapterLogging.hpp"

// RTDB & types
#include "FalconsRtDB2.hpp"


class cRefboxRelay
{
public:
    cRefboxRelay();
    ~cRefboxRelay();

    bool tick();
    void run();

private:
    // iteration-related
    double _frequency = 100.0;
    int _subsampleRtdbWrite = 1; // 100Hz
    int _subsampleMSLfeedback = 5; // 20Hz
    int _subsampleAttemptReconnect = 200; // only once in 2 seconds
    int _iteration = 0;

    // RTDB
    RtDB2 *_rtdb = NULL;
    RtDB2 *_rtdb_sim_team_B = NULL;
    T_REFBOX_CONFIG _refboxConfig;
    T_MATCH_STATE _matchState;
    T_MATCH_STATE _matchState_sim_team_B;

    // MSL feedback
    bool _doFeedbackLogging = true;
    RTDBAdapterLogging _logger;

    // refbox client
    CTCPIP_Client *_client = NULL;
    std::string _IPAddress;
    int _port = 28097;

private:
    void refboxCallBack(char const *command);
    void createClient();
    std::string teamColorString();
    std::string IPAddress();
};

#endif

