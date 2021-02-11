// Copyright 2016-2017 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * timerTest.cpp
 *
 *  Created on: Sep 24, 2016
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include <boost/shared_ptr.hpp>
#include "int/utilities/timer.hpp"

class TimerTest : public TeamplayTest { };

TEST_F(TimerTest, MoreThanZeroSecondsHaveElapsedAfterMicroSleep)
{
    timer timer;
    usleep(1);
    EXPECT_TRUE(timer.hasElapsed(0.0));
}

TEST_F(TimerTest, TenSecondsHaveNotElapsedSinceTimerStart)
{
    timer timer;
    EXPECT_FALSE(timer.hasElapsed(10.0));
}

TEST_F(TimerTest, NegativeSecondsHaveAlwaysElapsedSinceTimerStart)
{
    timer timer;
    EXPECT_TRUE(timer.hasElapsed(-1.0));
}

TEST_F(TimerTest, MultipleTimersDoNotInterfere)
{
    boost::shared_ptr<timer> timer1;
    boost::shared_ptr<timer> timer2;

    timer1.reset(new timer());
    timer2.reset(new timer());

    EXPECT_FALSE(timer1->hasElapsed(0.0005));
    EXPECT_FALSE(timer2->hasElapsed(0.0005));

    usleep(1000);

    EXPECT_TRUE(timer1->hasElapsed(0.0005));
    EXPECT_TRUE(timer2->hasElapsed(0.0005));

    timer2.reset(new timer());

    EXPECT_TRUE(timer1->hasElapsed(0.0005));
    EXPECT_FALSE(timer2->hasElapsed(0.0005));
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
