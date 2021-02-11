// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cByteArrayTyp.hpp
 *
 *  Created on: Aug 11, 2015
 *      Author: Tim Kouters
 */

#ifndef CBYTEARRAYTYP_HPP_
#define CBYTEARRAYTYP_HPP_

#include <stdio.h>
#include <stdint.h>
#include <vector>

namespace Facilities
{

class cByteArray
{
	public:
		cByteArray();
		~cByteArray();

		size_t getSize();
		void setData(std::vector<uint8_t> const &data);
		void getData(std::vector<uint8_t> &data);

	private:
		std::vector<uint8_t> _data;

		void deleteData();
};

} /* namespace Facilities */

#endif /* CBYTEARRAYTYP_HPP_ */
