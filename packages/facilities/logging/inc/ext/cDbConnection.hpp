 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDbConnection.hpp
 *
 * Interfacing with RTDB instances. Base class for cLogger and cPlayback.
 *
 *  Created on: Aug 12, 2018
 *      Author: Jan Feitsma
 */

#ifndef CDBCONNECTION_HPP_
#define CDBCONNECTION_HPP_


#include <vector>
#include <map>
#include "RtDB2Store.h"


#define RTDB2_STORAGE_PRODUCTION ("/tmp/rtdb2_storage")
#define RTDB2_STORAGE_PLAYBACK ("/tmp/rtdb2_playback")


class cDbConnection
{
  public:
    cDbConnection(std::string const &storage = RTDB2_STORAGE_PRODUCTION);
    ~cDbConnection();
    
  public:
    std::vector<int> _agentIds;
    std::map<int, RtDB2*> _rtdb; // key: agent id
    void connectRTDB(std::string const &storage);
    void disconnectRTDB();
    
};

#endif

