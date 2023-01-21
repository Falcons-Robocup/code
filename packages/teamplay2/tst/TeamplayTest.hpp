// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * TeamplayTest.hpp
 *
 *  Created on: Dec 13, 2016
 *      Author: Coen Tempelaars
 */

#ifndef TEAMPLAYTEST_HPP_
#define TEAMPLAYTEST_HPP_

/* Include testframework */
#include "gtest/gtest.h"
#include "gmock/gmock.h"

/* Include trace utility */
#include "tracing.hpp"

namespace teamplay {};

using namespace ::testing;
using namespace teamplay;

class TeamplayTest : public Test
{
public:
    TeamplayTest()
    {
        //traceRedirect::getInstance().setAllTracesToStdout();
        // does not exist anymore, teamplay specific TRACE was removed in favour of Falcons common TRACE
    };
};

#endif /* TEAMPLAYTEST_HPP_ */
