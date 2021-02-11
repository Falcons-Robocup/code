// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cAbstractObserverByteArray.cpp
 *
 *  Created on: Aug 31, 2015
 *      Author: Tim Kouters
 */

#include "ext/cAbstractObserverByteArray.hpp"

#include <stdexcept>

using namespace Facilities;

using std::runtime_error;


/*!
 * \brief Constructor of cAbstractOberserByteArray class
 *
 * This abstract class is used as an interface class.
 * Clients that want to receive data need to implement this
 * class themselves
 *
 */
cAbstractObserverByteArray::cAbstractObserverByteArray()
{

}

/*!
 * \brief Destructor of cAbstractOberserByteArray class
 *
 */
cAbstractObserverByteArray::~cAbstractObserverByteArray()
{

}

/*!
 * \brief New packet notification
 *
 * This function is called when the UDP receiver received a new
 * byte array of data.
 * This function needs to be re-implemented by the client
 *
 * \param[in] cByteArray data Array of bytes received
 *
 * \note Not re-implementing this function will result in throwing
 * an error.
 */
void cAbstractObserverByteArray::notifyNewPacket(cByteArray &data)
{
	throw runtime_error("Abstract class implementation of ByteArray observer called. Function needs to be implemented by client");
}
