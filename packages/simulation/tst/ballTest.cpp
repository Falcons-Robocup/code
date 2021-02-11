// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballTest.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>

#include "int/ball.hpp"


class ABall : public ::testing::Test
{
public:
    Ball ball;
};


TEST_F(ABall, HasPosition)
{
    auto position = Point3D(0.5, 1.0, 1.5);
    ball.setPosition(position);
    EXPECT_EQ(position, ball.getPosition());
}

TEST_F(ABall, HasVelocity)
{
    auto velocity = Vector3D(0.1, 0.2, 0.3);
    ball.setVelocity(velocity);
    EXPECT_EQ(velocity, ball.getVelocity());
}

TEST_F(ABall, CanBeTeleportedWithoutSpecifyingHeight)
{
    ball.teleport(0.2, 0.4);
    EXPECT_EQ(Point3D(0.2, 0.4, 0.0), ball.getPosition());
}

TEST_F(ABall, CanBeTeleported)
{
    ball.teleport(0.2, 0.4, 0.6);
    EXPECT_EQ(Point3D(0.2, 0.4, 0.6), ball.getPosition());
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
