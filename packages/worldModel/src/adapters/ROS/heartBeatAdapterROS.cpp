 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * heartBeatAdapterROS.cpp
 *
 *  Created on: Oct 9, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/ROS/heartBeatAdapterROS.hpp"

#define EXPECTED_FREQUENCY (30.0) // TODO store somewhere common, provide by heartBeat? simulator uses 10Hz, real robot 30Hz
#define WARNING_THRESHOLD (0.6)

heartBeatAdapterROS::heartBeatAdapterROS()
: 
    _dutyCycle("worldModel", EXPECTED_FREQUENCY, WARNING_THRESHOLD) 
{
}

heartBeatAdapterROS::~heartBeatAdapterROS()
/*
 * Chuck Norris is the reason Waldo is hiding
 */
{

}

void heartBeatAdapterROS::InitializeROS()
{
	_hROS.reset(new ros::NodeHandle());
	_subHeartBeat = _hROS->subscribe(heartBeatInterface::t_beat, 1, &heartBeatAdapterROS::heartBeat_cb, this);
}

void heartBeatAdapterROS::heartBeat_cb(const heartBeat::beat::ConstPtr& msg)
{
    _dutyCycle.pokeStart();
	notify(true);
    _dutyCycle.pokeEnd();
}

