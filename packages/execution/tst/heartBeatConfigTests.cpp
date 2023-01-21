// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heartBeatConfigTests.cpp
 *
 *  Created on: Nov 10, 2015
 *      Author: Tim Kouters
 */

#include <gtest/gtest.h>

#include "int/config/cHeartBeatConfig.hpp"

TEST(TestSuiteWorldModelConfig, configSetInitial)
{
	/* Setup */

    /* Execution */


	/* Verification */
	ASSERT_FLOAT_EQ(30.0, cHeartBeatConfig::getInstance().getUpdateFrequency());
}

TEST(TestSuiteWorldModelConfig, configSet01)
{
	/* Setup */
	cHeartBeatConfig::getInstance().setUpdateFrequency(0.01);


    /* Execution */


	/* Verification */
	ASSERT_FLOAT_EQ(0.01, cHeartBeatConfig::getInstance().getUpdateFrequency());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
