 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPathPlanningNames.hpp
 *
 *  Created on: Dec 21, 2015
 *      Author: Coen Tempelaars
 */

#ifndef PATHPLANNINGNODENAMES_HPP_
#define PATHPLANNINGNODENAMES_HPP_

#include <string>

namespace PathPlanningNodeNames
{
const static std::string pathplanning_nodename = "PathPlanningNode";
}

namespace PathPlanningInterface
{
const static std::string s_pathplanning_move_at_speed      = "s_pathplanning_move_at_speed";
const static std::string s_pathplanning_move_then_turn     = "s_pathplanning_move_then_turn";
const static std::string s_pathplanning_move_while_turning = "s_pathplanning_move_while_turning";
const static std::string s_pathplanning_turn               = "s_pathplanning_turn";
const static std::string s_pathplanning_turn_then_move     = "s_pathplanning_turn_then_move";
const static std::string s_pathplanning_stop               = "s_pathplanning_stop";
}

#endif /* PATHPLANNINGNODENAMES_HPP_ */
