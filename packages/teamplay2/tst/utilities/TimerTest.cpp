// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * TimerTest.cpp
 *
 *  Created on: Sep 24, 2016
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include <boost/shared_ptr.hpp>
#include "int/utilities/Timer.hpp"

class TimerTest : public TeamplayTest { };

TEST_F(TimerTest, MoreThanZeroSecondsHaveElapsedAfterMicroSleep)
{
    Timer timer;
    usleep(1);
    EXPECT_TRUE(timer.hasElapsed(0.0));
}

TEST_F(TimerTest, TenSecondsHaveNotElapsedSinceTimerStart)
{
    Timer timer;
    EXPECT_FALSE(timer.hasElapsed(10.0));
}

TEST_F(TimerTest, NegativeSecondsHaveAlwaysElapsedSinceTimerStart)
{
    Timer timer;
    EXPECT_TRUE(timer.hasElapsed(-1.0));
}

TEST_F(TimerTest, MultipleTimersDoNotInterfere)
{
    boost::shared_ptr<Timer> timer1;
    boost::shared_ptr<Timer> timer2;

    timer1.reset(new Timer());
    timer2.reset(new Timer());

    EXPECT_FALSE(timer1->hasElapsed(0.0005));
    EXPECT_FALSE(timer2->hasElapsed(0.0005));

    usleep(1000);

    EXPECT_TRUE(timer1->hasElapsed(0.0005));
    EXPECT_TRUE(timer2->hasElapsed(0.0005));

    timer2.reset(new Timer());

    EXPECT_TRUE(timer1->hasElapsed(0.0005));
    EXPECT_FALSE(timer2->hasElapsed(0.0005));
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
