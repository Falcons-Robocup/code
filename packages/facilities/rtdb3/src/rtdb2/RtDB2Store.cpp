 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include "RtDB2Store.h"

RtDB2* RtDB2Store::getRtDB2(int db_identifier)
{
    return getRtDBInstance( std::make_pair(db_identifier, "") );
}
RtDB2* RtDB2Store::getRtDB2(int db_identifier, char team_char)
{
    std::string path = "";
    if (team_char == 'B')
    {
        path = RTDB2_SIM_TEAM_B_PATH;
    }
    return getRtDBInstance( std::make_pair(db_identifier, path) );
}
RtDB2* RtDB2Store::getRtDB2(int db_identifier, std::string path)
{
    return getRtDBInstance( std::make_pair(db_identifier, path) );
}
RtDB2* RtDB2Store::getRtDBInstance(const RtDB2InstanceKey& key)
{
    // find key.
    // if not found, create a new rtdb instance and add to the map under the key.
    // if found, return the existing rtdb instance
    auto it = _rtdbInstances.find(key);

    if (it != _rtdbInstances.end())
    {
        return it->second;
    }
    else
    {
        // rtdb instance not found.
        // create instance and add to the map
        RtDB2* newInstance = 0;
        if (key.second == "")
        {
            newInstance = new RtDB2(key.first);
        }
        else
        {
            newInstance = new RtDB2(key.first, key.second);
        }

        _rtdbInstances[key] = newInstance;

        return newInstance;
    }
}
