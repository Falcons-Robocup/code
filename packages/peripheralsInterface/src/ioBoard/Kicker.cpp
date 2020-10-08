 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Kicker.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Edwin Schreuder
 */

#include "falconsCommon.hpp"
#include "FalconsRtDB2.hpp"
#include "ConfigRTDBAdapter.hpp"

#include "int/ioBoard/Kicker.hpp"

ConfigRTDBAdapter<T_CONFIG_PERIPHERALSINTERFACE_KICKER>* _configAdapterKicker;

Kicker::Kicker(IoBoard &ioBoard) :
		ioBoard(ioBoard) {

    _configAdapterKicker = new ConfigRTDBAdapter<T_CONFIG_PERIPHERALSINTERFACE_KICKER>(CONFIG_PERIPHERALSINTERFACE_KICKER);
    std::string configFileKicker = determineConfig("Kicker");
    _configAdapterKicker->loadYAML(configFileKicker);
    _configAdapterKicker->setConfigUpdateCallback( std::bind(&Kicker::updateConfiguration, this) );

}

Kicker::~Kicker() {
    delete _configAdapterKicker;
}

void Kicker::updateConfiguration()
{

    T_CONFIG_PERIPHERALSINTERFACE_KICKER config;
    _configAdapterKicker->get(config);

    _leverMaxHeight = config.leverMaxHeight;
	_leverSpeed = config.leverSpeed;
	ioBoard.setLeverSpeed(((unsigned char) _leverSpeed) + 100);
}

void Kicker::move(float leverHeight) {

    // Clip to <0, _leverMaxHeight>
	if (leverHeight > _leverMaxHeight) {
		leverHeight = _leverMaxHeight;
	}
	else if (leverHeight < 0) {
		leverHeight = 0;
	}

	ioBoard.setHeight((unsigned char) leverHeight);
}

void Kicker::shoot(float shootPower) {
	ioBoard.setShoot((unsigned char) shootPower);
}

void Kicker::home() {
	ioBoard.setHome();
}
