 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * shootplanningnames.h
 *
 *  Created on: May 26, 2014
 *      Author: Tim Kouters
 */

#ifndef SHOOTPLANNINGNAMES_H_
#define SHOOTPLANNINGNAMES_H_

#include <string>

namespace ShootPlanningNodeNames
{

const static std::string shootplanning_nodename = "ShootPlanningNode";
}

namespace ShootPlanningInterface
{
const static std::string s_kick_position = "s_kick_position";
const static std::string s_kick_speed = "s_kick_speed";
const static std::string s_pass = "s_pass";
const static std::string s_self_pass = "s_self_pass";
const static std::string s_shoot = "s_shoot";
const static std::string s_lobshot = "s_lobshot";
const static std::string s_shootplanning_set_active = "s_shootplanning_set_active";
const static std::string s_shootplanning_get_active = "s_shootplanning_get_active";
}

#endif /* SHOOTPLANNINGNAMES_H_ */
