// Copyright 2018-2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cQueryInterface.hpp
 *
 *  Created on: Apr 24, 2018
 *      Author: Erik Kouters / Coen Tempelaars
 */

#ifndef CQUERYINTERFACE_HPP_
#define CQUERYINTERFACE_HPP_

#include "MP_WorldModelInterface.hpp"

class cQueryInterface
{
  public:
    cQueryInterface() {};
    virtual ~cQueryInterface() {};

    void connect(MP_WorldModelInterface* wm);

    double timeToBall(const uint8_t robotID) const;
    
  private:
    MP_WorldModelInterface* _wm;

};

#endif /* CQUERYINTERFACE_HPP_ */

