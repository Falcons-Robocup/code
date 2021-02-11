// Copyright 2016-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * identifierGenerator.hpp
 *
 *  Created on: Sep 4, 2016
 *      Author: Tim Kouters
 */

#ifndef IDENTIFIERGENERATOR_HPP_
#define IDENTIFIERGENERATOR_HPP_

#include <stddef.h>
#include <stdint.h>

#include "uniqueObjectID.hpp"

class identifierGenerator
{
	public:
		identifierGenerator() {};
		identifierGenerator(uint8_t robotID);
		~identifierGenerator();
		uniqueObjectID getUniqueID();

	private:
		uint8_t _robotID;
		size_t _uniqueID;
};

#endif /* IDENTIFIERGENERATOR_HPP_ */
