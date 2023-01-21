// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * velocityControlTestDefaults.hpp
 *
 *  Created on: November 2019
 *      Author: Jan Feitsma
 */

#ifndef VELOCITYCONTROL_TESTDEFAULTS_HPP_
#define VELOCITYCONTROL_TESTDEFAULTS_HPP_

#include "int/VelocityControl.hpp"


// common setup and some config values (we normally don't want to be sensitive to production yamls)
VelocityControl velocityControlSetup(vcCFI *vcConfig, ppCFI *ppConfig, exCFI *exConfig, OutputInterface *output = NULL);
VelocityControl defaultVelocityControlSetup();



#endif
