// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cAbstractObserverByteArray.hpp
 *
 *  Created on: Aug 11, 2015
 *      Author: Tim Kouters
 */

#ifndef CABSTRACTBYTEARRAYOBSERVER_HPP_
#define CABSTRACTBYTEARRAYOBSERVER_HPP_

#include "cByteArray.hpp"

namespace Facilities
{

/* Byte array observer class */
class cAbstractObserverByteArray
{
    public:
        cAbstractObserverByteArray();
        virtual ~cAbstractObserverByteArray();

        virtual void notifyNewPacket(cByteArray &data);

    private:

};

} /* namespace Facilities */

#endif /* CABSTRACTBYTEARRAYOBSERVER_HPP_ */
