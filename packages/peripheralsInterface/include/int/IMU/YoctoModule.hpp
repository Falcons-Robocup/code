// Copyright 2016 Edwin (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * YoctoModule.hpp
 *
 *  Created on: 19 feb. 2016
 *      Author: Edwin Schreuder
 */

#ifndef YOCTOMODULE_HPP_
#define YOCTOMODULE_HPP_

#include <string>

#include "yocto_api.h"

using namespace std;

class YoctoModule
{
	private:
		void registerHub(const string hub);
		void findModule(const string module_name, const string module_type);

	protected:
		YModule * module;

	public:
		YoctoModule(const string module_name, const string module_type, const string hub = "usb");
		~YoctoModule();

		bool isOnline();

		void reboot();

		void setLedOn();
		void setLedBeacon();
		void setLedIntensity(int intensity);

		string getInfo();
		string getSerialNumber();
};

#endif /* YOCTOMODULE_HPP_ */
