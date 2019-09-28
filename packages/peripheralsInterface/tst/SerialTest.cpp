 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

