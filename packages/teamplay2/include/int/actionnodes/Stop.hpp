// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0

#ifndef STOP_HPP
#define STOP_HPP

#include "int/actionnodes/AbstractAction.hpp"

// NOTE: This action is not available in the tree

class Stop : public AbstractAction
{
  public:
    Stop()
    {
    }

    void stop() { AbstractAction::stop(); }

    behTreeReturnEnum execute(bool onStarted) { return behTreeReturnEnum::PASSED; };

};

#endif