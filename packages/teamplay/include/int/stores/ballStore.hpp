// Copyright 2016 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballStore.hpp
 *
 *  Created on: Aug 11, 2016
 *      Author: Coen Tempelaars
 */

#ifndef BALLSTORE_HPP_
#define BALLSTORE_HPP_

#include "int/types/ball.hpp"


namespace teamplay
{

class ballStore {
public:
    static ballStore& getInstance()
    {
        static ballStore instance;
        return instance;
    }

    static ball& getBall()
    {
        return getInstance()._theBall;
    }

private:
    ballStore();
    virtual ~ballStore();
    ballStore(ballStore const&); // Don't implement
    void operator= (ballStore const&); // Don't implement

    ball _theBall;
};


} /* namespace teamplay */

#endif /* BALLSTORE_HPP_ */
