 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * filterTest.cpp
 *
 *  Created on: December 2019
 *      Author: Jan Feitsma
 */


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
using namespace ::testing;

// Setup
#define NUMERICAL_TOLERANCE 1e-4

// Include functionality to be tested
#include "ext/LinearInterpolator.hpp" 
#include "ext/LeastSquaresInterpolator.hpp"



TEST(filterTest, LinearInterpolator_interpolate)
{
    // Arrange
    auto f = LinearInterpolator();
    f.feed(30.0, 100.0);
    f.feed(70.0, 200.0);

    // Act
    float y30 = f.evaluate(30.0);
    float y70 = f.evaluate(70.0);
    float y50 = f.evaluate(50.0);
    float y60 = f.evaluate(60.0);

    // Assert
    EXPECT_NEAR(y30, 100.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(y70, 200.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(y50, 150.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(y60, 175.0, NUMERICAL_TOLERANCE);
}

TEST(filterTest, LinearInterpolator_extrapolate)
{
    // Arrange
    auto f = LinearInterpolator();
    f.feed(30.0, 100.0);
    f.feed(70.0, 200.0);
    f.feed(80.0, 210.0);

    // Act
    float y20  = f.evaluate(20.0);
    float y10  = f.evaluate(10.0);
    float y90  = f.evaluate(90.0);

    // Assert
    EXPECT_NEAR(y20,  75.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(y10,  50.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(y90, 220.0, NUMERICAL_TOLERANCE);
}

TEST(filterTest, LinearInterpolator_insufficient_data)
{
    // Arrange
    auto f = LinearInterpolator();
    f.feed(30.0, 100.0);

    // Act and assert
    EXPECT_ANY_THROW(f.evaluate(20.0));
}

TEST(filterTest, LeastSquaresInterpolator_interpolate_linear)
{
    // Arrange
    auto f = LeastSquaresInterpolator(1);
    f.feed(30.0,  97.5);
    f.feed(40.0, 130.0);
    f.feed(60.0, 170.0);
    f.feed(70.0, 202.5);
    // Linear regression: y = 2.5 * x + 25.0

    // Act
    float y50  = f.evaluate(50.0);
    float y60  = f.evaluate(60.0);

    // Assert
    EXPECT_NEAR(y50, 150.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(y60, 175.0, NUMERICAL_TOLERANCE);
}

TEST(filterTest, LeastSquaresInterpolator_extrapolate_linear)
{
    // Arrange
    auto f = LeastSquaresInterpolator(1);
    f.feed(30.0,  97.5);
    f.feed(40.0, 130.0);
    f.feed(60.0, 170.0);
    f.feed(70.0, 202.5);
    // Linear regression: y = 2.5 * x + 25.0

    // Act
    float y20  = f.evaluate(20.0);
    float y10  = f.evaluate(10.0);
    float y90  = f.evaluate(90.0);

    // Assert
    EXPECT_NEAR(y20,  75.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(y10,  50.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(y90, 250.0, NUMERICAL_TOLERANCE);
}

TEST(filterTest, LeastSquaresInterpolator_interpolate_quadratic)
{
    // Arrange
    auto f = LeastSquaresInterpolator(2);
    f.feed(30.0, 200.0);
    f.feed(40.0, 125.0);
    f.feed(60.0, 125.0);
    f.feed(70.0, 200.0);
    // Quadratic regression: y = 725 - 25*x + 0.25*x*x

    // Act
    float y30  = f.evaluate(30.0);
    float y50  = f.evaluate(50.0);

    // Assert
    EXPECT_NEAR(y30, 200.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(y50, 100.0, NUMERICAL_TOLERANCE);
}

TEST(filterTest, LeastSquaresInterpolator_extrapolate_quadratic)
{
    // Arrange
    auto f = LeastSquaresInterpolator(2);
    f.feed(30.0, 200.0);
    f.feed(40.0, 125.0);
    f.feed(60.0, 125.0);
    f.feed(70.0, 200.0);
    // Quadratic regression: y = 725 - 25*x + 0.25*x*x

    // Act
    float y10  = f.evaluate(10.0);
    float y100 = f.evaluate(100.0);

    // Assert
    EXPECT_NEAR(y10,  500.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(y100, 725.0, NUMERICAL_TOLERANCE);
}

TEST(filterTest, LeastSquaresInterpolator_insufficient_data_0)
{
    // Arrange
    auto f = LeastSquaresInterpolator(1);

    // Act and assert
    EXPECT_ANY_THROW(f.evaluate(20.0));
}

TEST(filterTest, LeastSquaresInterpolator_insufficient_data_1)
{
    // Arrange
    auto f = LeastSquaresInterpolator(1);
    f.feed(30.0, 100.0);

    // Act
    float y10  = f.evaluate(10.0);
    // where our linear interpolator would give and error,
    // the least-squares solution is just the same as given data

    // Assert
    EXPECT_NEAR(y10, 100.0, NUMERICAL_TOLERANCE);
}

TEST(filterTest, LeastSquaresInterpolator_insufficient_data_2)
{
    // Arrange
    auto f = LeastSquaresInterpolator(2);
    f.feed(30.0, 100.0);
    f.feed(70.0, 200.0);

    // Act
    float y10  = f.evaluate(10.0);
    // where our linear interpolator would give and error,
    // the least-squares solution is a linear fit

    // Assert
    EXPECT_NEAR(y10, 50.0, NUMERICAL_TOLERANCE);
}

TEST(filterTest, LeastSquaresInterpolator_lots_of_data)
{
    // Fit a high-order polynomial through a line made of many data points
    // (So below matrix fit will have many rows and many columns)

    // Arrange
    int N = 100000;
    int m = 8;
    auto f = LeastSquaresInterpolator(m);
    float a = 3.4;
    float b = -123.7;
    for (int it = 0; it < N; ++it)
    {
        f.feed(it, a * it + b);
    }

    // Act
    float x = 10.0;
    float y = f.evaluate(x); // 199ms on laptop bakpao with N=1e5 and m=8

    // Assert
    EXPECT_NEAR(y, a*x+b, 1.0);
}

TEST(filterTest, LeastSquaresInterpolator_range_selection)
{
    // Arrange
    auto f = LeastSquaresInterpolator(1, 16.0);
    f.feed(30.0,  97.5);
    f.feed(40.0, 130.0);
    f.feed(60.0, 170.0);
    f.feed(70.0, 202.5);

    // Act and assert
    EXPECT_NEAR(f.evaluate(52.5), 155.86, 1.0);
    EXPECT_NEAR(f.evaluate(60.0), 173.78, 1.0);
    EXPECT_NEAR(f.evaluate(80.0), 234.07, 1.0); // linear extrapolation
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

