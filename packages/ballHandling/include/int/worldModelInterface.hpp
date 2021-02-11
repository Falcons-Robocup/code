// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * worldModelInterface.hpp
 *
 * Communicate to worldModel if ballHandlers are lifted, which suggests ballPossession.
 * WorldModel verifies with a 2nd opinion, currently using vision, in future possibly using a dedicated sensor.
 *
 *  Created on: Mar 4, 2018
 *      Author: Jan Feitsma
 */

#ifndef WORLDMODELINTERFACE_HPP_
#define WORLDMODELINTERFACE_HPP_

class worldModelInterface
{
  public:
    worldModelInterface() {};
    virtual ~worldModelInterface() {};
    
    virtual void setBallHandlersLifted(bool lifted) = 0;
    
};

#endif /* WORLDMODELINTERFACE_HPP_ */

