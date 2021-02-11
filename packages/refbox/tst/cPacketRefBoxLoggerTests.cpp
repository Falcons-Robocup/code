// Copyright 2016 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPacketRefBoxLoggerTests.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Tim Kouters
 */

#include <gtest/gtest.h>

#include <string>

#include "int/logger/cPacketRefboxLogger.hpp"

using namespace packetRefboxLogger;

TEST(TestSuitePacketRefboxLogger, defaultPacket)
{
	/* Setup */
	cPacketRefboxLogger packet;

    /* Execution */
	std::string jsonMsg;
	packet.getSerialized(jsonMsg);

	/* Verification */
	ASSERT_GT(jsonMsg.size(), 0);

	printf("string received: %s \n", jsonMsg.data());
}


/*
 * Main entry
 */
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
