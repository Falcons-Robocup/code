 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cConfigRules.hpp
 *
 *  Created on: May 24, 2016
 *      Author: Martijn van Veen
 */

#ifndef CCONFIGRULES_HPP_
#define CCONFIGRULES_HPP_

#include <dynamic_reconfigure/server.h>
#include <teamplay/teamplayRulesConfig.h>


class cConfigRules
{
public:
    static cConfigRules& getInstance()
    {
        static cConfigRules instance;
        return instance;
    }

private:
    boost::shared_ptr< dynamic_reconfigure::Server<teamplay::teamplayRulesConfig> > _srv;
    dynamic_reconfigure::Server<teamplay::teamplayRulesConfig>::CallbackType _f;

    void loadConfigYaml();
    void reconfig_cb(teamplay::teamplayRulesConfig &config, uint32_t level);

    cConfigRules();
    ~cConfigRules();
    cConfigRules(cConfigRules const&); // Don't Implement
    void operator=(cConfigRules const&);	   // Don't implement
};


#endif /* CCONFIGRULES_HPP_ */
