 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 *  Created on: Dec 21, 2015
 *      Author: Jan Feitsma
 */

#ifndef NETWORK_PORTS_HPP_
#define NETWORK_PORTS_HPP_

#include <string>

namespace Facilities
{
namespace Network
{

static const int PORT_OFFSET_TEAM_A    = 10000;
static const int PORT_OFFSET_TEAM_B    = 20000;
static const int PORT_OFFSET_COACH     =     0;
static const int PORT_OFFSET_PER_ROBOT =  1000;
static const int PORT_OFFSET_SIMULATOR =   500;
// DANGER: do NOT change these values unless you also change udpInterface.py


// each robot has 1000 consecutive ports available
// which are used according to following enum
enum portEnum
{ 
    PORT_INVALID = 0,
    PORT_ROBOTCONTROL = 1,   // DANGER: do NOT change this particular value unless you also change udpInterface.py
    PORT_REFBOXRELAY,
    PORT_WORLDMODELSYNC,
    PORT_WORLDMODELSYNC_STD,
    PORT_TEAMPLAY_INTENTION,
    // insert new ports here
    PORT_DIAG_BASE
    // all ports below PORT_DIAG_BASE are reserved for diagnostics!
    // so do NOT add new ports here!
};


// wrapper to apply team- and robot offsets
// and return a unique port
int getPort(portEnum key, int robotNumber);



} /* namespace Network */
} /* namespace Facilities */

#endif /* NETWORK_PORTS_HPP_ */
