// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * uniqueObjectID.hpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#ifndef UNIQUEOBJECTID_HPP_
#define UNIQUEOBJECTID_HPP_

#include <stddef.h>
#include <stdint.h>

#include "RtDB2.h" // required for serialization


struct uniqueObjectID
{
    uint8_t robotID;
    size_t  uniqueID;

    uniqueObjectID() :
    	robotID(0),uniqueID(0) {}

    uniqueObjectID(const uint8_t rID,const size_t uID) :
    	robotID(rID),uniqueID(uID) {}

    /*
     * Equality operator needed for std::map functionality
     */
    bool operator == (const uniqueObjectID &obj) const;

    /*
     * Lesser operator needed for std::map::find functionality
     */
    bool operator < (const uniqueObjectID &obj) const;
    
    SERIALIZE_DATA_FIXED(robotID, uniqueID);
};

#endif
