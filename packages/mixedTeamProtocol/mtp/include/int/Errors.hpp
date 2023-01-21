// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_ERRORS_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_ERRORS_HPP_

namespace mtp
{

// use single bits so errors can be combined 
const int ERROR_UNINITIALIZED = 1;
const int ERROR_BAD_ROLE = 2;
const int ERROR_SHIRT_CONFLICT = 4;

} // end of namespace mtp

#endif
