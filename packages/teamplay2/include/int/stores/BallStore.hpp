// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * BallStore.hpp
 *
 *  Created on: Aug 11, 2016
 *      Author: Coen Tempelaars
 */

#ifndef BALLSTORE_HPP_
#define BALLSTORE_HPP_

#include "int/types/Ball.hpp"


namespace teamplay
{

class BallStore {
public:
    static BallStore& getInstance()
    {
        static BallStore instance;
        return instance;
    }

    static Ball& getBall()
    {
        return getInstance()._theBall;
    }

private:
    BallStore();
    virtual ~BallStore();
    BallStore(BallStore const&); // Don't implement
    void operator= (BallStore const&); // Don't implement

    Ball _theBall;
};


} /* namespace teamplay */

#endif /* BALLSTORE_HPP_ */
