// Copyright 2015-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldModelInterface.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef CWORLDMODELINTERFACE_HPP_
#define CWORLDMODELINTERFACE_HPP_

#include "int/types/cBallPossessionTypes.hpp"
#include "int/types/cRobotLocationTypes.hpp"

#include "int/worldModelInfo.hpp"

class cWorldModelInterface
{
public:
    static cWorldModelInterface& getInstance()
    {
        static cWorldModelInterface instance;
        return instance;
    }

    /*
     * Getter functionality (to be phased out)
     */
    void getBallPossession(ballPossession_struct_t &ballPossession);

    /*
     * Setter functionality (to be phased out)
     */
    void setBallPossession(ballPossession_struct_t const &ballPossession);

    /*
     * Store functionality for the worldmodel adapter
     */
    virtual void store (const teamplay::worldModelInfo&);

private:
    ballPossession_struct_t _ballPossession;

    cWorldModelInterface();
    ~cWorldModelInterface();
    cWorldModelInterface(cWorldModelInterface const&); // Don't Implement
    void operator=(cWorldModelInterface const&);	   // Don't implement
};

#endif /* CWORLDMODELINTERFACE_HPP_ */
