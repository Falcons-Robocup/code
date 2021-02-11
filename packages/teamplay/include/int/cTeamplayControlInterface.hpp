// Copyright 2016-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cTeamplayControlInterface.hpp
 *
 *  Created on: Jun 18, 2016
 *      Author: Jan Feitsma
 */

#ifndef CTEAMPLAYCONTROLINTERFACE_HPP_
#define CTEAMPLAYCONTROLINTERFACE_HPP_


#include <vector>
#include <string>
#include "int/adapters/RtdbAdapterControlOverride.hpp" // TODO: better to instantiate and connect in main()?

#include "tpOverrideState.hpp" // sharedTypes

class cTeamplayControlInterface
{
public:
    static cTeamplayControlInterface& getInstance()
    {
        static cTeamplayControlInterface instance;
        return instance;
    }
    void getOverrideState(tpOverrideState &overrideState);
    void setOverrideResult(tpOverrideResult const &overrideResult);

private:
    cTeamplayControlInterface();
    void reset(tpOverrideState &overrideState);
    RtdbAdapterControlOverride _rtdbAdapter;
};

#endif /* CTEAMPLAYCONTROLINTERFACE_HPP_ */
