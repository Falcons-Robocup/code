// Copyright 2017-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cByteArray.cpp
 *
 *  Created on: Aug 11, 2015
 *      Author: Tim Kouters
 */

#include "ext/cByteArray.hpp"

#include <stdlib.h>
#include <string.h>
#include "falconsCommon.hpp"


using namespace Facilities;


/*!
 * \brief Constructor for the Byte Array class
 *
 * This class is a representation of a byte array
 *
 */
cByteArray::cByteArray()
{
}

/*!
 * \brief Destructor for the byte array class
 *
 * \note Ensures the data within this class is freed
 */
cByteArray::~cByteArray()
{
	// no action needed here due to using vector
}

/*!
 * \brief Get the size of the byte array in this class
 *
 * \retval size_t size of the array in bytes
 *
 * \note Optional note
 */
size_t cByteArray::getSize()
{
	return _data.size();
}


/*!
 * \brief Get the data
 *
 * \param[out] std::vector<uint8_t> &data Reference to the buffer container
 *
 * \retval void None
 *
 * \note The output data itself is a pointer to the real data.
 *       Data is not copied.
 */
void cByteArray::getData(std::vector<uint8_t> &data)
{
    data = _data;
}

/*!
 * \brief Set byte array data
 *
 * This function copies the data.
 *
 * \param[in] std::vector<uint8_t> &data Reference to the buffer container
 */
void cByteArray::setData(std::vector<uint8_t> const &data)
{
    _data = data;
}


