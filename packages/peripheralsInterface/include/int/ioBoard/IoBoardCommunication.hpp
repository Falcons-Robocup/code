// Copyright 2017-2018 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * IoBoard.hpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_INT_IOBOARDCOMMUNICATION_HPP_
#define INCLUDE_INT_IOBOARDCOMMUNICATION_HPP_

#include <array>
#include <string>

#include "int/Serial.hpp"

class IoBoardCommunication {
public:
  using Data = array<unsigned char, 8>;

  IoBoardCommunication(std::string port_name);
  ~IoBoardCommunication();

  void set(unsigned char command, Data data);
  Data get(unsigned char command);

private:
  struct Package {
    unsigned char command;
    IoBoardCommunication::Data data;
  };

  Data do_transaction(unsigned char command, Data data);

  Package transceive_package(Package transmit_package);
  void transmit_package(Package package);
  Package receive_package();
  unsigned char calculate_checksum(vector<unsigned char> package);

  Serial serial;
};

#endif /* INCLUDE_INT_IOBOARDCOMMUNICATION_HPP_ */
