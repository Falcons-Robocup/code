// Copyright 2017 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cShootPlanningInterface.hpp
 *
 *  Created on: Nov 18, 2017
 *      Author: Jan Feitsma
 */

#ifndef CSHOOTPLANNINGINTERFACE_HPP_
#define CSHOOTPLANNINGINTERFACE_HPP_

class cShootPlanningInterface
{
  public:
    cShootPlanningInterface() {};
    virtual ~cShootPlanningInterface() {};

    virtual void prepareForShot(float distance, float z, bool doLob) = 0;
    virtual void executeShot() = 0;
    virtual void executePass(float distance) = 0;

};

#endif /* CSHOOTPLANNINGINTERFACE_HPP_ */

