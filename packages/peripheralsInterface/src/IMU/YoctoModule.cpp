 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
