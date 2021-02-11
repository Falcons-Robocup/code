// Copyright 2017 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * SerialTest.cpp
 *
 *  Created on: Apr 25, 2017
 *      Author: Edwin Schreuder
 */

#include "gtest/gtest.h"

#include "int/Serial.hpp"

#include <exception>
#include <string>
#include <pty.h>
#include <unistd.h>

using std::runtime_error;
using std::string;
using namespace ::testing;

class SerialTest : public Test {
public:
	int master_file_descriptor;
	int slave_file_descriptor;
	string slave_name;

private:
	virtual void SetUp() {
		char name[256];
		int result = openpty(&master_file_descriptor, &slave_file_descriptor, name, NULL, NULL);
		slave_name = string(name);
		if (result < 0) {
			throw runtime_error("Could not create virtual serial port.");
		}
	}

	virtual void TearDown() {

	}
};

TEST_F(SerialTest, instantiateTest) {
	Serial serial(slave_name, 115200, 0.0, 0);
}

//! Test to verify that on timeout the amount of bytes returned is 0.
TEST_F(SerialTest, timeoutTest) {
	Serial serial(slave_name, 115200, 0.001, 0);
	
	unsigned char data[256];
	int byte_count = serial.readPort(data, 256);
	
	ASSERT_EQ(byte_count, 0);
}

int main(int argc, char ** argv) {
	InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

