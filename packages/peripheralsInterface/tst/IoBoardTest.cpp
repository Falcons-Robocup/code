// Copyright 2018 Antonio Eleuterio (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * IoBoardTest.cpp
 *
 *  Created on: Apr 24, 2018
 *      Authors: Antonio Eleuterio
 *               Edwin Schreuder
 */

#include <iostream>
#include <vector>
#include <exception>
#include <string>
//#include <pty.h> TODO: check if this is necessary
#include <unistd.h>

#include "gtest/gtest.h"

#include "int/Serial.hpp"
#include "int/ioBoard/IoBoard.hpp"


using std::runtime_error;
using std::string;
using namespace ::testing;

class IoBoardTest : public Test
{
public:
  std::string portName = "/dev/ttyS0";
//  IoBoard ioBoard = IoBoard(portName);

private:
  virtual void SetUp()
  {
  }

  virtual void TearDown()
  {
  }
};

TEST_F(IoBoardTest, DISABLED_ttyS0Test)
{
  Serial serial(portName, 115200, 1.0, 0);

  vector<unsigned char> data = {0x5A, 0x08, 0x0d};

  EXPECT_NO_THROW({
    serial.writePort(data);
    data = serial.readPort(10);
  });

  std::cout << hex;
  for (vector<unsigned char>::iterator entry = data.begin(); entry < data.end(); entry++)
  {
    std::cout << *entry;
  }
  std::cout << std::dec;
  std::cout << std::endl;
}

///* Initializing the ioboard twice has to run without errors */
//TEST_F(IoBoardTest, DISABLED_doubleInitialization)
//{
//  EXPECT_NO_THROW({
//    ioBoard.initialize();
//    ioBoard.initialize();
//  });
//}

/* The IoBoard needs to be able to handle getStatus continually */
//TEST_F(IoBoardTest, DISABLED_getStatusDDoS)
//{
//  int ddos_amount = 100;
//  ioBoard.initialize();
//
//  IoBoard::Status status;
//
//  for(int i = 0; i < ddos_amount; i++)
//  {
//    EXPECT_NO_THROW({
//      status = ioBoard.getStatus();
//    });
//  }
//}

///* Test the shooting */
//TEST_F(IoBoardTest, DISABLED_setShoot)
//{
//  float shootPower = 50;
//
//  EXPECT_NO_THROW({
//    ioBoard.setShoot((unsigned char) shootPower);
//  });
//}

///* The IoBoard needs to be able to handle setHome continually */
//TEST_F(IoBoardTest, DISABLED_setHomeDDoS)
//{
//  int ddos_amount = 10;
//
//  for(int i = 0; i < ddos_amount; i++)
//  {
//    EXPECT_NO_THROW({
//      setHome();
//    });
//    //TODO: create a delay
//  }
//}

///* The IoBoard needs to be able to handle setHeight continually */
//TEST_F(IoBoardTest, DISABLED_setHeightDDoS)
//{
//  float heightMax = 255;
//  int ddos_amount = 10;
//
//  for(int i = 0; i < ddos_amount; i++)
//  {
//    EXPECT_NO_THROW({
//      ioBoard.setShoot((unsigned char) heightMax * i / ddos_amount);
//    });
//  }
//}

///* The IoBoard needs to be able to handle setLeverSpeed continually */
//TEST_F(IoBoardTest, DISABLED_setLeverSpeedDDoS)
//{
//  float speedMax = 255;
//  for(int i = 0; i < ddos_amount; i++)
//  {
//    EXPECT_NO_THROW({
//      ioBoard.setLeverSpeed((unsigned char) speedMax * i / ddos_amount);
//    });
//  }
//}

int main(int argc, char ** argv) {
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

