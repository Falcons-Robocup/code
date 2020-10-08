 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRtDBClient.hpp
 *
 * RtDB client facility, intended for consumer interfaces offered by the data producers.
 * e.g., WorldModel offers the cWorldModelClient interface to all consumers that need WM data (balls, obstacles, etc).
 * cWorldModelClient inherits the class cRtDBClient.
 *
 *  Created on: Oct 20, 2018
 *      Author: Erik Kouters
 */

#ifndef CRTDBCLIENT_HPP_
#define CRTDBCLIENT_HPP_

// system includes
#include <vector>
#include <map>

#include "falconsCommon.hpp" // getRobotNumber()
#include "RtDB2Store.h"

class cRtDBClient
{
public:

    int _myRobotId;
    std::vector<int> _agentIds;
    std::map<int, RtDB2*> _rtdb; // key: agent id

    cRtDBClient()
    {
        for (int i = 0; i <= MAX_ROBOTS; i++)
        {
            _agentIds.push_back(i);
        }
        _myRobotId = getRobotNumber();
        connectRTDB();
    }

    ~cRtDBClient()
    {
        disconnectRTDB();
    }

    void connectRTDB()
    {
        auto teamChar = getTeamChar();
        for (auto it = _agentIds.begin(); it != _agentIds.end(); ++it)
        {
            _rtdb[*it] = RtDB2Store::getInstance().getRtDB2(*it, teamChar);
        }
    }

    void disconnectRTDB()
    {
        for (auto it = _agentIds.begin(); it != _agentIds.end(); ++it)
        {
            // Do not close the RtDB connection
            //delete _rtdb[*it];
        }
    }
};

#endif //CRTDBCLIENT_HPP_

