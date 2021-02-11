// Copyright 2016 Edwin (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * YoctoModule.cpp
 *
 *  Created on: 16 feb. 2016
 *      Author: Edwin Schreuder
 */

#include <exception>
#include <iostream>
#include <string>

#include "int/IMU/YoctoModule.hpp"

#include "yocto_api.h"

using namespace std;

YoctoModule::YoctoModule(const string device_name, const string device_type, const string hub)
{
	registerHub(hub);
	findModule(device_name, device_type);
}

void YoctoModule::registerHub(const string hub)
{
	string error;

	if (yRegisterHub(hub, error) != YAPI_SUCCESS) {
		throw(runtime_error("Could not create hub '" + string(hub) + "' :" + error));
	}
	else {
		cout << "Registered to hub '"<< hub << "'." << endl;
	}
}

void YoctoModule::findModule(const string device_name, const string device_type)
{
	if (device_name != "") {
		module = yFindModule(device_name);
	}
	else {
		module = yFirstModule();
		while (module) {
			if (module->productName() == device_type) {
				break;
			}
		}
	}

	if (module == NULL) {
		throw runtime_error("Could not find " + device_type + " module.");
	}

	if (module->isOnline()) {
		cout << "Found " << device_type << " module (" << module->describe() << ")." << endl;
		module->setLuminosity(0);
		if (module->beacon() != Y_BEACON_ON) {
			module->setBeacon(Y_BEACON_ON);
			module->saveToFlash();
		}
		module->setBeacon(Y_BEACON_OFF);
	}
	else {
		throw runtime_error(device_type + " module is not connected.");
	}
}

YoctoModule::~YoctoModule()
{
	module->setBeacon(Y_BEACON_ON);
}

void YoctoModule::reboot()
{
	module->reboot(0);
}

void YoctoModule::setLedOn()
{
	module->setBeacon(Y_BEACON_OFF);
}

void YoctoModule::setLedBeacon()
{
	module->setBeacon(Y_BEACON_ON);
}

void YoctoModule::setLedIntensity(int intensity)
{
	if (intensity > 100)
	{
		intensity = 100;
	}
	module->setLuminosity(intensity);
}

string YoctoModule::getInfo()
{
	string settings;

	if (module->isOnline()) {
		settings =	"Product name:       " + module->productName() + '\n' + \
					"Product ID:         " + to_string(module->productId()) + '\n' + \
					"Product Release:    " + to_string(module->productRelease()) + '\n' + \
					"Name:               " + module->logicalName() + '\n' + \
					"Serial Number:      " + module->serialNumber() + '\n' + \
					"Firmware Release:   " + module->firmwareRelease() + '\n' + \
					"Luminosity:         " + to_string(module->luminosity()) + '\n' + \
					"LED:                " + (module->beacon() == Y_BEACON_ON ? "Beacon" : "On") + '\n' + \
					"USB Current:        " + to_string(module->usbCurrent()) + '\n' + \
					"Uptime:             " + to_string(module->upTime());
	}

	return (settings);
}

string YoctoModule::getSerialNumber()
{
	return (module->get_serialNumber());
}

bool YoctoModule::isOnline()
{
	return (module->isOnline());
}
