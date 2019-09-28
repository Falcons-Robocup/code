 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
