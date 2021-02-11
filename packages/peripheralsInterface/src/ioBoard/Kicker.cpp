// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
