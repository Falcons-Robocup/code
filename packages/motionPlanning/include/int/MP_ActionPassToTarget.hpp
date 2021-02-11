// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionPassToTarget.hpp
 *
 *  Created on: Nov 26, 2017
 *      Author: Jan Feitsma
 */

#ifndef MP_ACTIONPASSTOTARGET_HPP_
#define MP_ACTIONPASSTOTARGET_HPP_

#include "MP_ActionShootAtTarget.hpp"


// reuse shootAtTarget but with different configuration 

class MP_ActionPassToTarget: public MP_ActionShootAtTarget
{
  public:
    MP_ActionPassToTarget();

  private:
    virtual void unpackParameters();
    virtual void getSpecificConfig();
    virtual void executeShot();
    
};

#endif /* MP_ACTIONPASSTOTARGET_HPP_ */

