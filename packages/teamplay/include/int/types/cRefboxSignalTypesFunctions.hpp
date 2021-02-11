// Copyright 2016-2019 martijn van veen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRefboxSignalTypesFunctions.hpp
 * Contains simple conversion functions that are only used in some callers, to avoid w_unused function from many includes
 *
 *  Created on: July 7, 2016
 *      Author: Martijn van Veen
 */

#ifndef CREFBOXSIGNALTYPESFUNCTIONS_HPP_
#define CREFBOXSIGNALTYPESFUNCTIONS_HPP_

#include <iostream>
#include <string>
#include <map>

#include "cRefboxSignalTypes.hpp"

static refboxSignalEnum refBoxSignalStrToEnum(const std::string enumString)
{
	if(refboxSignalMapping.find(enumString) == refboxSignalMapping.end())
	{
		std::cout << enumString << std::endl;
	}
	return	refboxSignalMapping.at(enumString);
}


#endif /* CREFBOXSIGNALTYPESFUNCTIONS_HPP_ */


