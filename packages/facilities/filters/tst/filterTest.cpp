// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

TEST(filterTest, LeastSquaresInterpolator_fit_parabola)
{
    auto f = LeastSquaresInterpolator(2);
    f.feed(1.0,  1.0);
    f.feed(2.0,  4.0);
    f.feed(3.0,  9.0);
    f.feed(4.0, 16.0);

    std::vector<double> coeffs = f.calculate_polynomial();

    EXPECT_NEAR(coeffs[0], 0.0, 1e-6);
    EXPECT_NEAR(coeffs[1], 0.0, 1e-6);
    EXPECT_NEAR(coeffs[2], 1.0, 1e-6);
}

int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    return r;
}

